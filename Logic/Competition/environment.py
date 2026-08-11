"""Reimplementación local del contrato de Indoor Exploration Competition.

Las coordenadas son siempre ``(row, col)`` y las etiquetas son idénticas al
entorno oficial: libre=0, desconocido=0.5, ocupado=1.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from heapq import heappop, heappush
import importlib.util
from math import ceil, cos, log10, pi, sin
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from .information_gain import CircularUnknownInformationGain, InformationGainMethod


@dataclass(slots=True)
class CompetitionConfig:
    lidar_range: float = 15.0
    num_laser: int = 2500
    pixel_per_meter: int = 10
    comm_range: float = 15.0
    attenuation_constant: float = 10.0
    transmitted_power: float = 20.0
    path_loss_exponent: float = 3.5
    power_threshold: float = -80.0
    pd_size: int = 200
    num_robots: int = 3
    max_steps: int = 1000
    start_pose: tuple[int, int] = (15, 15)  # input convention: x, y
    other_traj_threshold: float = 5.0
    other_intent_threshold: float = 10.0
    relay_trigger: str = "periodic"
    relay_period: int = 200
    final_relay: bool = True
    relay_transfer: bool = True

    def policy_options(self) -> SimpleNamespace:
        return SimpleNamespace(**asdict(self))


@dataclass(slots=True)
class Observation:
    combined_obs_map: np.ndarray
    pose: np.ndarray
    unreported_mask: np.ndarray
    delegated_mask: np.ndarray
    pose_lists_of_others: dict[str, np.ndarray]
    intents_of_others: dict[str, np.ndarray]
    base_pose: np.ndarray
    pd_size: int
    connected_robot_ids: list[int] = field(default_factory=list)


class InvalidGoalError(ValueError):
    pass


class BasePolicy:
    def decide(self, obs: Observation, collect_opts: Any):
        raise NotImplementedError

    def should_relay(self, obs: Observation, collect_opts: Any, t: int, max_steps: int):
        if collect_opts.final_relay and max_steps - t < collect_opts.relay_period:
            path = astar(inflate_map(obs.combined_obs_map), obs.pose, obs.base_pose)
            if path is not None and max_steps - t <= ceil((len(path) - 1) / 3) + 10:
                return "final_relay"
        if collect_opts.relay_trigger == "periodic" and t > 0 and t % collect_opts.relay_period == 0:
            return "relay"
        return None

    def decide_relay_handoff(self, obs: Observation, collect_opts: Any):
        candidates = []
        own_distance = np.linalg.norm(obs.pose - obs.base_pose)
        for robot_id in obs.connected_robot_ids:
            poses = obs.pose_lists_of_others.get(f"robot{robot_id}")
            if poses is not None and len(poses) and np.linalg.norm(poses[-1] - obs.base_pose) < own_distance:
                candidates.append((np.linalg.norm(poses[-1] - obs.pose), robot_id))
        return min(candidates)[1] if candidates else None


def load_policy(path: str | Path) -> BasePolicy:
    spec = importlib.util.spec_from_file_location("submitted_policy", path)
    if spec is None or spec.loader is None:
        raise ValueError(f"No se pudo importar la policy: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    policy_class = getattr(module, "Policy", None)
    if policy_class is None or not issubclass(policy_class, BasePolicy):
        raise TypeError("La policy debe definir Policy(BasePolicy)")
    return policy_class()


def load_competition_map(path: str | Path, padding: int = 200) -> np.ndarray:
    raw = np.load(Path(path), allow_pickle=False)
    if raw.ndim != 2 or not set(np.unique(raw)).issubset({0, 254, 255}):
        raise ValueError("occ_map.npy debe ser 2D y contener únicamente 0/254/255")
    rows, cols = (raw.shape[0] + 1) // 2, (raw.shape[1] + 1) // 2
    padded = np.pad(raw, ((0, rows * 2 - raw.shape[0]), (0, cols * 2 - raw.shape[1])), constant_values=0)
    reduced = padded.reshape(rows, 2, cols, 2).min(axis=(1, 3))
    result = np.where(reduced == 0, 1.0, 0.0)
    return np.pad(result, int(padding), constant_values=1.0)


def _line(start: np.ndarray, end: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    r0, c0, r1, c1 = map(int, (*start, *end))
    count = max(abs(r1 - r0), abs(c1 - c0)) + 1
    return np.rint(np.linspace(r0, r1, count)).astype(int), np.rint(np.linspace(c0, c1, count)).astype(int)


def visible_cells(gt_map: np.ndarray, pose: np.ndarray, radius: int, rays: int) -> tuple[np.ndarray, np.ndarray]:
    free: set[tuple[int, int]] = set()
    hits: set[tuple[int, int]] = set()
    for angle in np.linspace(0.0, 2.0 * pi, int(rays), endpoint=False):
        endpoint = pose + np.array((sin(angle), cos(angle))) * radius
        rr, cc = _line(pose, endpoint)
        valid = (rr >= 0) & (cc >= 0) & (rr < gt_map.shape[0]) & (cc < gt_map.shape[1])
        for row, col in zip(rr[valid], cc[valid]):
            cell = (int(row), int(col))
            if gt_map[cell] == 1:
                hits.add(cell)
                break
            free.add(cell)
    return np.asarray(sorted(free), dtype=int).reshape(-1, 2), np.asarray(sorted(hits), dtype=int).reshape(-1, 2)


def inflate_map(observed: np.ndarray) -> np.ndarray:
    dilated = observed.copy()
    occupied = observed > 0.5
    expanded = occupied.copy()
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            expanded |= np.roll(np.roll(occupied, dr, axis=0), dc, axis=1)
    expanded[[0, -1], :] = True
    expanded[:, [0, -1]] = True
    dilated[expanded] = 1
    distance = np.zeros(dilated.shape, dtype=np.int16)
    frontier = dilated == 1
    unseen = ~frontier
    for level in range(1, 10):
        grown = frontier.copy()
        grown[1:] |= frontier[:-1]; grown[:-1] |= frontier[1:]
        grown[:, 1:] |= frontier[:, :-1]; grown[:, :-1] |= frontier[:, 1:]
        ring = grown & unseen
        distance[ring] = level
        unseen &= ~ring
        frontier = grown
    distance[unseen] = 10
    costs = np.clip(10 - distance, 1, 10).astype(np.float32)
    costs[dilated == 1] = np.inf
    return costs


def astar(costs: np.ndarray, start: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
    source, target = tuple(map(int, start)), tuple(map(int, goal))
    if not (0 <= target[0] < costs.shape[0] and 0 <= target[1] < costs.shape[1]) or not np.isfinite(costs[target]):
        return None
    queue = [(0.0, source)]
    parents: dict[tuple[int, int], tuple[int, int] | None] = {source: None}
    scores = {source: 0.0}
    while queue:
        _, current = heappop(queue)
        if current == target:
            path = []
            while current is not None:
                path.append(current)
                current = parents[current]
            return np.asarray(path[::-1], dtype=int)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nxt = current[0] + dr, current[1] + dc
            if not (0 <= nxt[0] < costs.shape[0] and 0 <= nxt[1] < costs.shape[1]) or not np.isfinite(costs[nxt]):
                continue
            score = scores[current] + float(costs[nxt])
            if score < scores.get(nxt, np.inf):
                scores[nxt], parents[nxt] = score, current
                heappush(queue, (score + abs(nxt[0]-target[0]) + abs(nxt[1]-target[1]), nxt))
    return None


def get_frontiers(observed: np.ndarray, threshold: int = 10) -> list[np.ndarray]:
    unknown = observed == 0.5
    near_unknown = np.zeros_like(unknown)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            near_unknown |= np.roll(np.roll(unknown, dr, axis=0), dc, axis=1)
    edges = near_unknown & (observed == 0)
    visited = np.zeros_like(edges)
    result = []
    for seed in np.argwhere(edges):
        seed_tuple = tuple(seed)
        if visited[seed_tuple]:
            continue
        stack = [seed_tuple]
        visited[seed_tuple] = True
        region = []
        while stack:
            row, col = stack.pop()
            region.append((row, col))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nxt = row + dr, col + dc
                    if (0 <= nxt[0] < edges.shape[0] and 0 <= nxt[1] < edges.shape[1]
                            and edges[nxt] and not visited[nxt]):
                        visited[nxt] = True
                        stack.append(nxt)
        points = np.asarray(region)
        if len(points) > threshold:
            center = points.mean(axis=0)
            result.append(points[np.argmin(np.linalg.norm(points - center, axis=1))])
    return result


class NearestFrontierPolicy(BasePolicy):
    def decide(self, obs: Observation, collect_opts: Any):
        candidates = get_frontiers(obs.combined_obs_map)
        for goal in sorted(candidates, key=lambda p: np.linalg.norm(p - obs.pose)):
            if astar(inflate_map(obs.combined_obs_map), obs.pose, goal) is not None:
                return goal
        return None


@dataclass(frozen=True, slots=True)
class UtilityWeights:
    information_gain: float = 0.50
    travel_cost: float = 0.25
    redundancy: float = 0.20
    relay_risk: float = 0.05

    def __post_init__(self) -> None:
        values = (self.information_gain, self.travel_cost,
                  self.redundancy, self.relay_risk)
        if any(value < 0 or not np.isfinite(value) for value in values):
            raise ValueError("Los pesos deben ser finitos y no negativos")
        if not np.isclose(sum(values), 1.0, atol=1e-9):
            raise ValueError("wIG + wC + wR + wL debe ser igual a 1")

    @property
    def label(self) -> str:
        return (f"{self.information_gain:.2f}/{self.travel_cost:.2f}/"
                f"{self.redundancy:.2f}/{self.relay_risk:.2f}")


class WeightedUtilityPolicy(BasePolicy):
    """Policy interpretable basada en cuatro features normalizados [0, 1]."""

    def __init__(self, weights: UtilityWeights | None = None,
                 information_gain: InformationGainMethod | None = None) -> None:
        self.weights = weights or UtilityWeights()
        self.information_gain_method = information_gain or CircularUnknownInformationGain()

    def decide(self, obs: Observation, collect_opts: Any):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        distances = np.linalg.norm(candidates - obs.pose, axis=1)
        distance_norm = distances / max(float(distances.max()), 1.0)
        gains = np.asarray([self.information_gain_method.calculate(obs.combined_obs_map, point)
                            for point in candidates], dtype=float)
        gain_norm = gains / max(float(gains.max()), 1.0)
        redundancy = np.asarray([self._redundancy(obs, point, collect_opts)
                                 for point in candidates])
        relay_risk = np.asarray([self._relay_risk(obs, point, collect_opts)
                                 for point in candidates])
        w = self.weights
        utility = (w.information_gain * gain_norm
                   - w.travel_cost * distance_norm
                   - w.redundancy * redundancy
                   - w.relay_risk * relay_risk)
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-utility):
            if astar(costs, obs.pose, candidates[index]) is not None:
                return candidates[index]
        return None

    @staticmethod
    def _redundancy(obs: Observation, point: np.ndarray, collect_opts: Any) -> float:
        distances = []
        for values in (*obs.pose_lists_of_others.values(), *obs.intents_of_others.values()):
            if values is not None and len(values):
                distances.append(float(np.linalg.norm(np.asarray(values)-point, axis=1).min()))
        if not distances:
            return 0.0
        scale = max(collect_opts.other_intent_threshold * collect_opts.pixel_per_meter, 1.0)
        return float(np.clip(1.0 - min(distances)/scale, 0.0, 1.0))

    @staticmethod
    def _relay_risk(obs: Observation, point: np.ndarray, collect_opts: Any) -> float:
        distance_m = float(np.linalg.norm(point-obs.base_pose)) / collect_opts.pixel_per_meter
        return float(np.clip(distance_m / max(collect_opts.comm_range, 1e-9), 0.0, 1.0))


@dataclass(slots=True)
class CompetitionRobot:
    id: int
    pose: np.ndarray
    policy: BasePolicy
    start_delay: int
    combined_obs_map: np.ndarray
    unreported_mask: np.ndarray
    delegated_mask: np.ndarray
    pose_list: np.ndarray
    intent: np.ndarray | None = None
    locked_goal: np.ndarray | None = None
    behavior_mode: str = "explore"
    pose_lists_of_others: dict[str, np.ndarray] = field(default_factory=dict)
    intents_of_others: dict[str, np.ndarray] = field(default_factory=dict)


class CompetitionWorld:
    def __init__(self, occ_map_path: str | Path, config: CompetitionConfig | None = None,
                 policy: BasePolicy | None = None, return_policy=None,
                 handoff_policy=None):
        self.config = config or CompetitionConfig()
        self.options = self.config.policy_options()
        self.occ_map = load_competition_map(occ_map_path, self.config.pd_size)
        start = np.array((self.config.start_pose[1] + self.config.pd_size,
                          self.config.start_pose[0] + self.config.pd_size), dtype=int)
        self.base_pose = start.copy()
        self.base_obs_map = np.full(self.occ_map.shape, 0.5)
        shape = self.occ_map.shape
        selected_policy = policy or NearestFrontierPolicy()
        self.return_policy = return_policy
        self.handoff_policy = handoff_policy
        self.handoff_count = 0
        self.robots = [CompetitionRobot(i + 1, start.copy(), selected_policy, i * 5,
            np.full(shape, 0.5), np.zeros(shape, bool), np.zeros(shape, bool), start[None, :].copy())
            for i in range(self.config.num_robots)]
        self.comm_graph = np.zeros((len(self.robots), len(self.robots)), bool)
        self.base_comm_graph = np.zeros(len(self.robots), bool)
        self.timestep = 0

    @property
    def coverage(self) -> float:
        p = self.config.pd_size
        return float(np.mean(self.base_obs_map[p:-p, p:-p] != 0.5))

    def live_observation_map(self) -> np.ndarray:
        """Unión visual de lo conocido por robots; no altera el scoring."""
        result = np.full(self.occ_map.shape, 0.5)
        for robot in self.robots:
            result[robot.combined_obs_map == 0] = 0
        for robot in self.robots:
            result[robot.combined_obs_map == 1] = 1
        return result

    def observation_for(self, robot: CompetitionRobot) -> Observation:
        connected = [int(i) + 1 for i in np.where(self.comm_graph[robot.id - 1])[0]]
        return Observation(robot.combined_obs_map.copy(), robot.pose.copy(), robot.unreported_mask.copy(),
            robot.delegated_mask.copy(), {k:v.copy() for k,v in robot.pose_lists_of_others.items()},
            {k:v.copy() for k,v in robot.intents_of_others.items()}, self.base_pose.copy(),
            self.config.pd_size, connected)

    def _can_communicate(self, first: np.ndarray, second: np.ndarray) -> bool:
        pixels = float(np.linalg.norm(first - second))
        distance = max(pixels / self.config.pixel_per_meter, 1e-6)
        if distance > self.config.comm_range:
            return False
        rr, cc = _line(first, second)
        walls = int(np.sum(self.occ_map[rr, cc] == 1))
        power = self.config.transmitted_power - 10*self.config.path_loss_exponent*log10(distance) - walls*self.config.attenuation_constant
        return power > self.config.power_threshold

    @staticmethod
    def _fuse(first: np.ndarray, second: np.ndarray) -> np.ndarray:
        result = np.full(first.shape, 0.5)
        result[(first == 0) | (second == 0)] = 0
        result[(first == 1) | (second == 1)] = 1
        return result

    def _observe(self, robot: CompetitionRobot) -> None:
        free, hits = visible_cells(self.occ_map, robot.pose,
            round(self.config.lidar_range * self.config.pixel_per_meter), self.config.num_laser)
        new_free = robot.combined_obs_map[free[:,0], free[:,1]] == 0.5 if len(free) else []
        new_hits = robot.combined_obs_map[hits[:,0], hits[:,1]] == 0.5 if len(hits) else []
        if len(free):
            robot.unreported_mask[free[new_free,0], free[new_free,1]] = True
            writable = robot.combined_obs_map[free[:,0], free[:,1]] != 1
            robot.combined_obs_map[free[writable,0], free[writable,1]] = 0
        if len(hits):
            robot.unreported_mask[hits[new_hits,0], hits[new_hits,1]] = True
            robot.combined_obs_map[hits[:,0], hits[:,1]] = 1

    def _communicate(self) -> None:
        n = len(self.robots)
        self.comm_graph[:] = False
        for i in range(n):
            for j in range(i + 1, n):
                if self._can_communicate(self.robots[i].pose, self.robots[j].pose):
                    self.comm_graph[i,j] = self.comm_graph[j,i] = True
                    a, b = self.robots[i], self.robots[j]
                    fused = self._fuse(a.combined_obs_map, b.combined_obs_map)
                    a.combined_obs_map = fused.copy(); b.combined_obs_map = fused.copy()
                    a.pose_lists_of_others[f"robot{b.id}"] = b.pose_list.copy()
                    b.pose_lists_of_others[f"robot{a.id}"] = a.pose_list.copy()
                    if a.intent is not None: b.intents_of_others[f"robot{a.id}"] = a.intent.copy()
                    if b.intent is not None: a.intents_of_others[f"robot{b.id}"] = b.intent.copy()
    def _base_communicate(self) -> None:
        for i, robot in enumerate(self.robots):
            connected = self._can_communicate(robot.pose, self.base_pose)
            self.base_comm_graph[i] = connected
            if connected:
                fused = self._fuse(robot.combined_obs_map, self.base_obs_map)
                robot.combined_obs_map = fused.copy(); self.base_obs_map = fused
                robot.unreported_mask[:] = False; robot.delegated_mask[:] = False
                if robot.behavior_mode == "relay": robot.behavior_mode = "explore"

    def _choose_goal(self, robot: CompetitionRobot, costs: np.ndarray) -> np.ndarray | None:
        goal = robot.policy.decide(self.observation_for(robot), self.options)
        if goal is not None:
            try: goal = np.asarray(goal, dtype=int).reshape(2)
            except (TypeError, ValueError) as error: raise InvalidGoalError(f"Robot {robot.id}: meta malformada") from error
            if astar(costs, robot.pose, goal) is None:
                raise InvalidGoalError(f"Robot {robot.id}: meta inválida o inalcanzable {tuple(goal)}")
            return goal
        for candidate in sorted(get_frontiers(robot.combined_obs_map), key=lambda p: np.linalg.norm(p-robot.pose)):
            if astar(costs, robot.pose, candidate) is not None: return candidate
        return None

    def _plan(self, robot: CompetitionRobot) -> np.ndarray:
        obs = self.observation_for(robot)
        if robot.behavior_mode == "explore":
            mode = (self.return_policy.decide(obs, self.options, self.timestep,
                                              self.config.max_steps)
                    if self.return_policy is not None else
                    robot.policy.should_relay(obs, self.options, self.timestep,
                                              self.config.max_steps))
            if mode in ("relay", "final_relay"): robot.behavior_mode = mode
        costs = inflate_map(robot.combined_obs_map)
        if robot.behavior_mode == "explore":
            if robot.locked_goal is not None and np.linalg.norm(robot.locked_goal-robot.pose) < 10: robot.locked_goal = None
            if robot.locked_goal is None or not np.isfinite(costs[tuple(robot.locked_goal)]):
                robot.locked_goal = self._choose_goal(robot, costs)
            path = astar(costs, robot.pose, robot.locked_goal) if robot.locked_goal is not None else None
        else:
            robot.locked_goal = None
            path = None
            # Reutiliza el plan de retorno ya validado; evita ejecutar múltiples
            # A* por robot en cada frame mientras el mapa no bloquee la ruta.
            if (self.return_policy is not None and robot.intent is not None
                    and len(robot.intent) and np.array_equal(robot.intent[-1], self.base_pose)):
                matches = np.where(np.all(robot.intent == robot.pose, axis=1))[0]
                if len(matches):
                    suffix = robot.intent[int(matches[-1]):]
                    if all(np.isfinite(costs[tuple(point)]) for point in suffix):
                        path = suffix
            if path is None:
                path = (self.return_policy.plan_return(
                            obs, self.options, self.config.max_steps-self.timestep)
                        if self.return_policy is not None else
                        astar(costs, robot.pose, self.base_pose))
            if robot.behavior_mode == "relay" and self.config.relay_transfer:
                target_id = (self.handoff_policy.decide(obs, self.options)
                             if self.handoff_policy is not None else
                             self.return_policy.decide_handoff(obs, self.options)
                             if self.return_policy is not None else
                             robot.policy.decide_relay_handoff(obs, self.options))
                target = next((r for r in self.robots if r.id == target_id), None)
                if target is not None and target.id != robot.id:
                    self.handoff_count += 1
                    transferred = robot.unreported_mask.copy()
                    robot.delegated_mask |= transferred
                    robot.unreported_mask &= ~transferred
                    robot.behavior_mode = "explore"
                    target.behavior_mode = "relay"
                    target.unreported_mask |= transferred
                    target.delegated_mask &= ~transferred
        if path is None: path = robot.pose[None,:]
        robot.intent = path
        return path[min(3, len(path)-1)].copy()

    def step(self) -> None:
        if self.timestep >= self.config.max_steps: return
        for robot in self.robots: self._observe(robot)
        self._communicate()
        for robot in self.robots:
            if self.timestep >= robot.start_delay:
                robot.pose = self._plan(robot)
                robot.pose_list = np.concatenate((robot.pose_list, robot.pose[None,:]))
        self._base_communicate()
        self.timestep += 1
