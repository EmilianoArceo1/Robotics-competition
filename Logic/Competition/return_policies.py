"""Policies independientes para decidir retornos y relevos hacia la base."""

from __future__ import annotations

from abc import ABC, abstractmethod
from math import ceil

import numpy as np

from .environment import Observation, astar, get_frontiers, inflate_map


class ReturnPolicy(ABC):
    """Contrato del algoritmo de retorno, separado de la exploración."""

    @abstractmethod
    def decide(self, obs: Observation, options, timestep: int, max_steps: int):
        """Devuelve ``relay``, ``final_relay`` o ``None``."""

    def decide_handoff(self, obs: Observation, options):
        own_distance = np.linalg.norm(obs.pose-obs.base_pose)
        candidates = []
        for robot_id in obs.connected_robot_ids:
            poses = obs.pose_lists_of_others.get(f"robot{robot_id}")
            if poses is not None and len(poses):
                peer = np.asarray(poses)[-1]
                if np.linalg.norm(peer-obs.base_pose) < own_distance:
                    candidates.append((np.linalg.norm(peer-obs.pose), robot_id))
        return min(candidates)[1] if candidates else None

    def plan_return(self, obs: Observation, options, remaining_steps: int):
        """Plan de retorno; por defecto utiliza el A* directo histórico."""
        return astar(inflate_map(obs.combined_obs_map), obs.pose, obs.base_pose)

    @staticmethod
    def return_steps(obs: Observation) -> int | None:
        path = astar(inflate_map(obs.combined_obs_map), obs.pose, obs.base_pose)
        return None if path is None else ceil((len(path)-1)/3)


class PeriodicReturnPolicy(ReturnPolicy):
    """Baseline compatible con el retorno histórico del simulador."""

    def decide(self, obs, options, timestep, max_steps):
        steps = self.return_steps(obs)
        remaining = max_steps-timestep
        if options.final_relay and steps is not None and remaining < options.relay_period:
            if remaining <= steps+10:
                return "final_relay"
        if options.relay_trigger == "periodic" and timestep > 0:
            if timestep % options.relay_period == 0:
                return "relay"
        return None


class DeadlineReturnPolicy(ReturnPolicy):
    """Explora al máximo y vuelve según tiempo de ruta con margen proporcional."""

    def __init__(self, safety_factor: float = 1.35, reserve_steps: int = 4) -> None:
        self.safety_factor = float(safety_factor)
        self.reserve_steps = int(reserve_steps)

    def decide(self, obs, options, timestep, max_steps):
        steps = self.return_steps(obs)
        if steps is None:
            return None
        deadline = ceil(self.safety_factor*steps)+self.reserve_steps
        return "final_relay" if max_steps-timestep <= deadline else None


class PayloadAdaptiveReturnPolicy(DeadlineReturnPolicy):
    """Regresa antes cuando acumula suficiente información no reportada."""

    def __init__(self, payload_cells: int = 1800, distance_m: float = 8.0) -> None:
        super().__init__(1.30, 5)
        self.payload_cells = int(payload_cells)
        self.distance_m = float(distance_m)

    def decide(self, obs, options, timestep, max_steps):
        final = super().decide(obs, options, timestep, max_steps)
        if final:
            return final
        payload = int(np.count_nonzero(obs.unreported_mask | obs.delegated_mask))
        distance = float(np.linalg.norm(obs.pose-obs.base_pose))/options.pixel_per_meter
        if payload >= self.payload_cells and distance >= self.distance_m:
            return "relay"
        return None


class LinkAwareReturnPolicy(DeadlineReturnPolicy):
    """Usa carga, distancia de comunicación y conectividad para retornar."""

    def __init__(self, payload_cells: int = 1200, comm_fraction: float = .72) -> None:
        super().__init__(1.45, 6)
        self.payload_cells = int(payload_cells)
        self.comm_fraction = float(comm_fraction)

    def decide(self, obs, options, timestep, max_steps):
        final = super().decide(obs, options, timestep, max_steps)
        if final:
            return final
        payload = int(np.count_nonzero(obs.unreported_mask | obs.delegated_mask))
        distance = float(np.linalg.norm(obs.pose-obs.base_pose))/options.pixel_per_meter
        link_limit = self.comm_fraction*options.comm_range
        disconnected = not obs.connected_robot_ids
        if payload >= self.payload_cells and distance >= link_limit and disconnected:
            return "relay"
        return None


class JustInTimeReturnPolicy(DeadlineReturnPolicy):
    """Vuelve en el último instante viable según la ruta A* actual."""

    def __init__(self) -> None:
        super().__init__(1.0, 0)


class EfficientPeriodicReturnPolicy(JustInTimeReturnPolicy):
    """Conserva entregas periódicas pero elimina el margen final ocioso."""

    def decide(self, obs, options, timestep, max_steps):
        final = super().decide(obs, options, timestep, max_steps)
        if final:
            return final
        if options.relay_trigger == "periodic" and timestep > 0:
            if timestep % options.relay_period == 0:
                return "relay"
        return None


class SelectiveCourierReturnPolicy(JustInTimeReturnPolicy):
    """Hace la entrega periódica sólo si transporta una carga significativa."""

    def __init__(self, minimum_payload: int = 300) -> None:
        super().__init__()
        self.minimum_payload = int(minimum_payload)

    def decide(self, obs, options, timestep, max_steps):
        final = super().decide(obs, options, timestep, max_steps)
        if final:
            return final
        payload = int(np.count_nonzero(obs.unreported_mask | obs.delegated_mask))
        periodic = (options.relay_trigger == "periodic" and timestep > 0
                    and timestep % options.relay_period == 0)
        return "relay" if periodic and payload >= self.minimum_payload else None


class ValueDensityReturnPolicy(JustInTimeReturnPolicy):
    """Retorna si las celdas entregables justifican el coste de la ruta."""

    def __init__(self, cells_per_return_step: float = 35.0,
                 minimum_distance_m: float = 6.0) -> None:
        super().__init__()
        self.cells_per_return_step = float(cells_per_return_step)
        self.minimum_distance_m = float(minimum_distance_m)

    def decide(self, obs, options, timestep, max_steps):
        steps = self.return_steps(obs)
        if steps is None:
            return None
        if max_steps-timestep <= steps:
            return "final_relay"
        if steps == 0:
            return None
        payload = int(np.count_nonzero(obs.unreported_mask | obs.delegated_mask))
        distance = float(np.linalg.norm(obs.pose-obs.base_pose))/options.pixel_per_meter
        if distance >= self.minimum_distance_m and payload/steps >= self.cells_per_return_step:
            return "relay"
        return None


class FrontierDetourReturnPolicy(ReturnPolicy):
    """Base para retornos que exploran una frontera factible antes de volver."""

    detour_steps = 10

    def decide(self, obs, options, timestep, max_steps):
        direct = self.return_steps(obs)
        if direct is None:
            return None
        remaining = max_steps-timestep
        return "final_relay" if remaining <= direct+self.detour_steps else None

    def candidate_score(self, candidate, obs, direct_pixels: int) -> float:
        raise NotImplementedError

    def plan_return(self, obs, options, remaining_steps):
        costs = inflate_map(obs.combined_obs_map)
        direct = astar(costs, obs.pose, obs.base_pose)
        if direct is None:
            return None
        capacity = max(int(remaining_steps), 0)*3
        candidates = get_frontiers(obs.combined_obs_map)
        best = None
        best_score = -np.inf
        # Limitar candidatos cercanos evita convertir el retorno en un cuello de botella.
        ordered = sorted(candidates, key=lambda p: np.linalg.norm(p-obs.pose))[:8]
        for candidate in ordered:
            outward = astar(costs, obs.pose, candidate)
            home = astar(costs, candidate, obs.base_pose)
            if outward is None or home is None:
                continue
            pixels = len(outward)+len(home)-2
            if pixels > capacity:
                continue
            score = self.candidate_score(np.asarray(candidate), obs, pixels)
            if score > best_score:
                best_score = score
                best = np.concatenate((outward, home[1:]), axis=0)
        return best if best is not None else direct


class NearestFrontierReturnPolicy(FrontierDetourReturnPolicy):
    """Visita la frontera factible más cercana durante el retorno final."""

    detour_steps = 2

    def candidate_score(self, candidate, obs, direct_pixels):
        return -float(np.linalg.norm(candidate-obs.pose))


class GainSweepReturnPolicy(FrontierDetourReturnPolicy):
    """Prioriza la frontera con más celdas desconocidas alrededor."""

    detour_steps = 3

    def candidate_score(self, candidate, obs, direct_pixels):
        row, col = map(int, candidate)
        radius = 20
        region = obs.combined_obs_map[max(0,row-radius):row+radius+1,
                                      max(0,col-radius):col+radius+1]
        gain = int(np.count_nonzero(region == .5))
        return gain-.15*direct_pixels


class HomewardSweepReturnPolicy(FrontierDetourReturnPolicy):
    """Explora sólo fronteras que mantienen progreso geométrico hacia base."""

    detour_steps = 2

    def candidate_score(self, candidate, obs, direct_pixels):
        current_home = float(np.linalg.norm(obs.pose-obs.base_pose))
        candidate_home = float(np.linalg.norm(candidate-obs.base_pose))
        if candidate_home >= current_home:
            return -np.inf
        row, col = map(int, candidate)
        region = obs.combined_obs_map[max(0,row-15):row+16, max(0,col-15):col+16]
        gain = int(np.count_nonzero(region == .5))
        return gain+.25*(current_home-candidate_home)-.10*direct_pixels


def make_return_policy(name: str) -> ReturnPolicy:
    policies = {
        "periodic": PeriodicReturnPolicy,
        "deadline": DeadlineReturnPolicy,
        "payload_adaptive": PayloadAdaptiveReturnPolicy,
        "link_aware": LinkAwareReturnPolicy,
        "just_in_time": JustInTimeReturnPolicy,
        "efficient_periodic": EfficientPeriodicReturnPolicy,
        "selective_courier": SelectiveCourierReturnPolicy,
        "value_density": ValueDensityReturnPolicy,
        "nearest_frontier_return": NearestFrontierReturnPolicy,
        "gain_sweep_return": GainSweepReturnPolicy,
        "homeward_sweep_return": HomewardSweepReturnPolicy,
    }
    try:
        return policies[name]()
    except KeyError as error:
        raise ValueError(f"Return policy desconocida: {name}") from error
