"""Variantes NBV candidatas para evaluación experimental."""

from __future__ import annotations

import numpy as np

from .environment import (BasePolicy, NearestFrontierPolicy, UtilityWeights,
                          astar, get_frontiers, inflate_map)
from .information_gain import PotentialVisibilityInformationGain
from .next_best_view import NextBestViewPolicy
from .viewpoints import FrontierOcclusionViewpointGenerator


class AdaptiveNextBestViewPolicy(NextBestViewPolicy):
    """Adapta pesos según conocimiento, carga sin reportar y conectividad."""

    def weights_for(self, obs, collect_opts):
        known_ratio = float(np.mean(obs.combined_obs_map != .5))
        known_count = max(int(np.count_nonzero(obs.combined_obs_map != .5)), 1)
        unreported_ratio = float(np.count_nonzero(obs.unreported_mask)) / known_count
        disconnected = not obs.connected_robot_ids
        if unreported_ratio > .35 and disconnected:
            return UtilityWeights(.40, .15, .15, .30)
        if known_ratio < .20:
            return UtilityWeights(.65, .10, .20, .05)
        if known_ratio < .55:
            return UtilityWeights(.50, .20, .25, .05)
        return UtilityWeights(.35, .30, .25, .10)

    def should_relay(self, obs, collect_opts, t, max_steps):
        final = super().should_relay(obs, collect_opts, t, max_steps)
        if final == "final_relay":
            return final
        unreported = int(np.count_nonzero(obs.unreported_mask))
        distance_m = float(np.linalg.norm(obs.pose-obs.base_pose))/collect_opts.pixel_per_meter
        if unreported >= 1800 and distance_m > .55*collect_opts.comm_range:
            return "relay"
        return final


class TunedNextBestViewPolicy(NextBestViewPolicy):
    """NBV conservador seleccionado por búsqueda local reproducible."""

    def __init__(self, information_gain=None, viewpoint_generator=None) -> None:
        super().__init__(UtilityWeights(.50, .20, .25, .05),
                         information_gain, viewpoint_generator)


class PhasedHybridPolicy(TunedNextBestViewPolicy):
    """Nearest al inicio; NBV cuando el mapa observado ya tiene estructura."""

    def __init__(self, switch_known_ratio: float = .02, information_gain=None,
                 viewpoint_generator=None) -> None:
        super().__init__(information_gain, viewpoint_generator)
        self.switch_known_ratio = float(switch_known_ratio)
        self.nearest = NearestFrontierPolicy()

    def decide(self, obs, collect_opts):
        p = int(obs.pd_size)
        real_map = obs.combined_obs_map[p:-p, p:-p] if p else obs.combined_obs_map
        known_ratio = float(np.mean(real_map != .5))
        if known_ratio < self.switch_known_ratio:
            return self.nearest.decide(obs, collect_opts)
        return super().decide(obs, collect_opts)


class IntentAwareNearestPolicy(BasePolicy):
    """Nearest Frontier rápido con separación por trayectorias e intents."""

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        for index, candidate in enumerate(candidates):
            trajectory_distances = [np.linalg.norm(np.asarray(values)-candidate, axis=1).min()
                for values in obs.pose_lists_of_others.values() if values is not None and len(values)]
            intent_distances = [np.linalg.norm(np.asarray(values)-candidate, axis=1).min()
                for values in obs.intents_of_others.values() if values is not None and len(values)]
            if trajectory_distances and min(trajectory_distances) < collect_opts.other_traj_threshold*collect_opts.pixel_per_meter:
                scores[index] -= 1_000_000
            if intent_distances and min(intent_distances) < collect_opts.other_intent_threshold*collect_opts.pixel_per_meter:
                scores[index] -= 100_000
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-scores):
            if astar(costs, obs.pose, candidates[index]) is not None:
                return candidates[index]
        return None


class SoftIntentAwareNearestPolicy(BasePolicy):
    """Coordinación gradual que evita los saltos de la penalización binaria."""

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        trajectory_threshold = collect_opts.other_traj_threshold*collect_opts.pixel_per_meter
        intent_threshold = collect_opts.other_intent_threshold*collect_opts.pixel_per_meter
        for index, candidate in enumerate(candidates):
            trajectories = [np.linalg.norm(np.asarray(v)-candidate, axis=1).min()
                for v in obs.pose_lists_of_others.values() if v is not None and len(v)]
            intents = [np.linalg.norm(np.asarray(v)-candidate, axis=1).min()
                for v in obs.intents_of_others.values() if v is not None and len(v)]
            if trajectories:
                scores[index] -= .35*max(0.0, trajectory_threshold-min(trajectories))
            if intents:
                scores[index] -= .75*max(0.0, intent_threshold-min(intents))
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-scores):
            if astar(costs, obs.pose, candidates[index]) is not None:
                return candidates[index]
        return None


class RegretBoundedIntentPolicy(BasePolicy):
    """Evita metas reclamadas sólo cuando el desvío adicional es razonable."""

    def __init__(self, maximum_detour_pixels: float = 50.0) -> None:
        self.maximum_detour_pixels = float(maximum_detour_pixels)

    @staticmethod
    def _claimed(candidate, obs, collect_opts) -> bool:
        trajectory_threshold = collect_opts.other_traj_threshold*collect_opts.pixel_per_meter
        intent_threshold = collect_opts.other_intent_threshold*collect_opts.pixel_per_meter
        for values in obs.pose_lists_of_others.values():
            if values is not None and len(values):
                if np.linalg.norm(np.asarray(values)-candidate, axis=1).min() < trajectory_threshold:
                    return True
        for values in obs.intents_of_others.values():
            if values is not None and len(values):
                if np.linalg.norm(np.asarray(values)-candidate, axis=1).min() < intent_threshold:
                    return True
        return False

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        distances = np.linalg.norm(candidates-obs.pose, axis=1)
        order = np.argsort(distances)
        costs = inflate_map(obs.combined_obs_map)
        baseline = None
        baseline_distance = None
        for index in order:
            if astar(costs, obs.pose, candidates[index]) is not None:
                baseline = candidates[index]
                baseline_distance = float(distances[index])
                break
        if baseline is None:
            return None
        if not self._claimed(baseline, obs, collect_opts):
            return baseline
        limit = baseline_distance + self.maximum_detour_pixels
        for index in order:
            if distances[index] > limit:
                break
            candidate = candidates[index]
            if (not self._claimed(candidate, obs, collect_opts)
                    and astar(costs, obs.pose, candidate) is not None):
                return candidate
        return baseline


class SelectiveIntentNearestPolicy(BasePolicy):
    """Nearest con penalización binaria sobre señales coordinativas elegidas."""

    def __init__(self, *, use_trajectories: bool, use_intents: bool,
                 threshold_scale: float = 1.0) -> None:
        self.use_trajectories = bool(use_trajectories)
        self.use_intents = bool(use_intents)
        self.threshold_scale = float(threshold_scale)

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        trajectory_threshold = (collect_opts.other_traj_threshold
                                * collect_opts.pixel_per_meter*self.threshold_scale)
        intent_threshold = (collect_opts.other_intent_threshold
                            * collect_opts.pixel_per_meter*self.threshold_scale)
        for index, candidate in enumerate(candidates):
            if self.use_trajectories:
                distances = [np.linalg.norm(np.asarray(v)-candidate, axis=1).min()
                    for v in obs.pose_lists_of_others.values() if v is not None and len(v)]
                if distances and min(distances) < trajectory_threshold:
                    scores[index] -= 1_000_000
            if self.use_intents:
                distances = [np.linalg.norm(np.asarray(v)-candidate, axis=1).min()
                    for v in obs.intents_of_others.values() if v is not None and len(v)]
                if distances and min(distances) < intent_threshold:
                    scores[index] -= 100_000
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-scores):
            if astar(costs, obs.pose, candidates[index]) is not None:
                return candidates[index]
        return None


class TrajectoryDiversifiedNearestPolicy(SelectiveIntentNearestPolicy):
    """Ganadora proxy: exclusión compacta de trayectorias a 2.5 metros."""

    def __init__(self) -> None:
        super().__init__(use_trajectories=True, use_intents=False,
                         threshold_scale=.50)


class RecentTrailNearestPolicy(BasePolicy):
    """Evita únicamente las últimas poses ajenas, no toda su historia."""

    def __init__(self, history_length: int = 12, radius_m: float = 2.5) -> None:
        self.history_length = int(history_length)
        self.radius_m = float(radius_m)

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        threshold = self.radius_m*collect_opts.pixel_per_meter
        trails = [np.asarray(values)[-self.history_length:]
                  for values in obs.pose_lists_of_others.values()
                  if values is not None and len(values)]
        for index, candidate in enumerate(candidates):
            if trails and min(np.linalg.norm(trail-candidate, axis=1).min()
                              for trail in trails) < threshold:
                scores[index] -= 1_000_000
        return _first_reachable(candidates, scores, obs)


class VoronoiNearestPolicy(BasePolicy):
    """Prefiere fronteras cuya distancia propia sea menor que la de sus peers."""

    def __init__(self, ownership_margin_m: float = .5) -> None:
        self.ownership_margin_m = float(ownership_margin_m)

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        own_distance = np.linalg.norm(candidates-obs.pose, axis=1)
        scores = -own_distance
        peers = [np.asarray(values)[-1] for values in obs.pose_lists_of_others.values()
                 if values is not None and len(values)]
        margin = self.ownership_margin_m*collect_opts.pixel_per_meter
        if peers:
            peer_distance = np.min(
                np.stack([np.linalg.norm(candidates-peer, axis=1) for peer in peers]),
                axis=0,
            )
            scores[peer_distance + margin < own_distance] -= 1_000_000
        return _first_reachable(candidates, scores, obs)


class FrontierReservationPolicy(BasePolicy):
    """Reserva el final de intents y combina una exclusión local muy corta."""

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        recent_trails = [np.asarray(values)[-8:]
            for values in obs.pose_lists_of_others.values() if values is not None and len(values)]
        reservations = [np.asarray(values)[-1]
            for values in obs.intents_of_others.values() if values is not None and len(values)]
        for index, candidate in enumerate(candidates):
            if recent_trails and min(np.linalg.norm(v-candidate, axis=1).min()
                                     for v in recent_trails) < 2.0*collect_opts.pixel_per_meter:
                scores[index] -= 100_000
            if reservations and min(np.linalg.norm(v-candidate) for v in reservations) < 4.0*collect_opts.pixel_per_meter:
                scores[index] -= 1_000_000
        return _first_reachable(candidates, scores, obs)


def _trajectory_clearances(candidates, obs) -> np.ndarray:
    """Distancia de cada frontera al rastro conocido más cercano."""
    trails = [np.asarray(values) for values in obs.pose_lists_of_others.values()
              if values is not None and len(values)]
    if not trails:
        return np.full(len(candidates), np.inf)
    return np.min(np.stack([
        np.min(np.linalg.norm(candidates[:, None, :] - trail[None, :, :], axis=2), axis=1)
        for trail in trails
    ]), axis=0)


class ElasticTrajectoryPolicy(BasePolicy):
    """Aumenta gradualmente la separación cuando el mapa ya está maduro."""

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        p = int(obs.pd_size)
        real_map = obs.combined_obs_map[p:-p, p:-p] if p else obs.combined_obs_map
        known_ratio = float(np.mean(real_map != .5))
        # 2.0 m al inicio para no forzar rodeos; hasta 3.0 m al madurar el mapa.
        radius_m = 2.0 + min(known_ratio/.20, 1.0)
        clearance = _trajectory_clearances(candidates, obs)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        scores[clearance < radius_m*collect_opts.pixel_per_meter] -= 1_000_000
        return _first_reachable(candidates, scores, obs)


class ClearanceUtilityPolicy(BasePolicy):
    """Equilibra cercanía y separación sin un salto binario en 2.5 m."""

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        ppm = collect_opts.pixel_per_meter
        clearance = _trajectory_clearances(candidates, obs)
        scores = -np.linalg.norm(candidates-obs.pose, axis=1)
        finite = np.isfinite(clearance)
        # Una separación adicional puede compensar como máximo 1.25 m de viaje.
        scores[finite] += .50*np.minimum(clearance[finite], 2.5*ppm)
        scores[clearance < 1.5*ppm] -= 1_000_000
        return _first_reachable(candidates, scores, obs)


class DetourCappedTrajectoryPolicy(BasePolicy):
    """Diversifica sólo si la alternativa no exige un desvío excesivo."""

    def __init__(self, radius_m: float = 2.5, maximum_detour_m: float = 3.0) -> None:
        self.radius_m = float(radius_m)
        self.maximum_detour_m = float(maximum_detour_m)

    def decide(self, obs, collect_opts):
        candidates = get_frontiers(obs.combined_obs_map)
        if not candidates:
            return None
        candidates = np.asarray(candidates)
        distances = np.linalg.norm(candidates-obs.pose, axis=1)
        clearance = _trajectory_clearances(candidates, obs)
        costs = inflate_map(obs.combined_obs_map)
        order = np.argsort(distances)
        baseline = None
        baseline_distance = 0.0
        for index in order:
            if astar(costs, obs.pose, candidates[index]) is not None:
                baseline = candidates[index]
                baseline_distance = float(distances[index])
                if clearance[index] >= self.radius_m*collect_opts.pixel_per_meter:
                    return baseline
                break
        if baseline is None:
            return None
        limit = baseline_distance + self.maximum_detour_m*collect_opts.pixel_per_meter
        for index in order:
            if distances[index] > limit:
                break
            if (clearance[index] >= self.radius_m*collect_opts.pixel_per_meter
                    and astar(costs, obs.pose, candidates[index]) is not None):
                return candidates[index]
        return baseline


def _first_reachable(candidates, scores, obs):
    costs = inflate_map(obs.combined_obs_map)
    for index in np.argsort(-scores):
        if astar(costs, obs.pose, candidates[index]) is not None:
            return candidates[index]
    return None


class GainPerCostPolicy(NextBestViewPolicy):
    """Maximiza ganancia marginal por esfuerzo, con penalización coordinada."""

    def decide(self, obs, collect_opts):
        candidates = self.viewpoint_generator.generate(obs.combined_obs_map, obs.pose)
        if not len(candidates):
            return None
        distances = np.linalg.norm(candidates-obs.pose, axis=1)
        distance_norm = distances/max(float(distances.max()), 1.0)
        gain = np.asarray([self.information_gain.calculate(obs.combined_obs_map, p)
                           for p in candidates], dtype=float)
        gain_norm = gain/max(float(gain.max()), 1.0)
        redundancy = np.asarray([self._redundancy(obs, p, collect_opts) for p in candidates])
        base_distance = np.linalg.norm(candidates-obs.base_pose, axis=1)
        relay = np.clip(base_distance/(collect_opts.comm_range*collect_opts.pixel_per_meter), 0, 1)
        # Cociente suavizado: evita tanto metas triviales como viajes enormes.
        utility = gain_norm/(.20 + distance_norm) - .30*redundancy - .08*relay
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-utility)[:40]:
            if astar(costs, obs.pose, candidates[index]) is not None:
                return candidates[index]
        return None


class CoordinatedOcclusionPolicy(NextBestViewPolicy):
    """Favorece viewpoints de oclusión y separación explícita entre robots."""

    def decide(self, obs, collect_opts):
        candidates = self.viewpoint_generator.generate(obs.combined_obs_map, obs.pose)
        if not len(candidates):
            return None
        distances = np.linalg.norm(candidates-obs.pose, axis=1)
        travel = distances/max(float(distances.max()), 1.0)
        gain = np.asarray([self.information_gain.calculate(obs.combined_obs_map, p)
                           for p in candidates], dtype=float)
        gain /= max(float(gain.max()), 1.0)
        redundancy = np.asarray([self._redundancy(obs, p, collect_opts) for p in candidates])
        occupied = obs.combined_obs_map == 1
        near_wall = np.zeros_like(occupied)
        near_wall[1:] |= occupied[:-1]; near_wall[:-1] |= occupied[1:]
        near_wall[:,1:] |= occupied[:,:-1]; near_wall[:,:-1] |= occupied[:,1:]
        occlusion_bonus = np.asarray([1.0 if near_wall[tuple(p)] else 0.0 for p in candidates])
        utility = .55*gain - .15*travel - .25*redundancy + .05*occlusion_bonus
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-utility)[:40]:
            if astar(costs, obs.pose, candidates[index]) is not None:
                return candidates[index]
        return None


def make_advanced_policy(name: str, *, rays: int = 180, candidates: int = 160):
    information = PotentialVisibilityInformationGain(100, rays)
    generator = FrontierOcclusionViewpointGenerator(maximum_candidates=candidates)
    policies = {
        "adaptive": AdaptiveNextBestViewPolicy,
        "tuned": TunedNextBestViewPolicy,
        "gain_per_cost": GainPerCostPolicy,
        "coordinated": CoordinatedOcclusionPolicy,
        "hybrid_005": lambda **kwargs: PhasedHybridPolicy(.005, **kwargs),
        "hybrid_010": lambda **kwargs: PhasedHybridPolicy(.010, **kwargs),
        "hybrid_020": lambda **kwargs: PhasedHybridPolicy(.020, **kwargs),
        "intent_nearest": lambda **kwargs: IntentAwareNearestPolicy(),
        "soft_intent_nearest": lambda **kwargs: SoftIntentAwareNearestPolicy(),
        "regret_25": lambda **kwargs: RegretBoundedIntentPolicy(25),
        "regret_50": lambda **kwargs: RegretBoundedIntentPolicy(50),
        "regret_75": lambda **kwargs: RegretBoundedIntentPolicy(75),
        "intent_only": lambda **kwargs: SelectiveIntentNearestPolicy(
            use_trajectories=False, use_intents=True),
        "trajectory_only": lambda **kwargs: SelectiveIntentNearestPolicy(
            use_trajectories=True, use_intents=False),
        "trajectory_050": lambda **kwargs: SelectiveIntentNearestPolicy(
            use_trajectories=True, use_intents=False, threshold_scale=.50),
        "trajectory_diversified": lambda **kwargs: TrajectoryDiversifiedNearestPolicy(),
        "recent_trail": lambda **kwargs: RecentTrailNearestPolicy(),
        "voronoi_nearest": lambda **kwargs: VoronoiNearestPolicy(),
        "frontier_reservation": lambda **kwargs: FrontierReservationPolicy(),
        "elastic_trajectory": lambda **kwargs: ElasticTrajectoryPolicy(),
        "clearance_utility": lambda **kwargs: ClearanceUtilityPolicy(),
        "detour_capped": lambda **kwargs: DetourCappedTrajectoryPolicy(),
        "trajectory_075": lambda **kwargs: SelectiveIntentNearestPolicy(
            use_trajectories=True, use_intents=False, threshold_scale=.75),
        "trajectory_125": lambda **kwargs: SelectiveIntentNearestPolicy(
            use_trajectories=True, use_intents=False, threshold_scale=1.25),
        "intent_tight": lambda **kwargs: SelectiveIntentNearestPolicy(
            use_trajectories=False, use_intents=True, threshold_scale=.70),
    }
    try:
        return policies[name](information_gain=information,
                              viewpoint_generator=generator)
    except KeyError as error:
        raise ValueError(f"Policy avanzada desconocida: {name}") from error
