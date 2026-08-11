"""Policy Next-Best-View con candidatos más amplios que centros de frontera."""

from __future__ import annotations

import numpy as np

from .environment import (BasePolicy, Observation, UtilityWeights, astar,
                          inflate_map)
from .information_gain import (InformationGainMethod,
                               PotentialVisibilityInformationGain)
from .viewpoints import (FrontierOcclusionViewpointGenerator,
                         ViewpointGenerator)


class NextBestViewPolicy(BasePolicy):
    def __init__(self, weights: UtilityWeights | None = None,
                 information_gain: InformationGainMethod | None = None,
                 viewpoint_generator: ViewpointGenerator | None = None) -> None:
        self.weights = weights or UtilityWeights()
        self.information_gain = information_gain or PotentialVisibilityInformationGain()
        self.viewpoint_generator = viewpoint_generator or FrontierOcclusionViewpointGenerator()

    def decide(self, obs: Observation, collect_opts):
        candidates = self.viewpoint_generator.generate(obs.combined_obs_map, obs.pose)
        if not len(candidates):
            return None
        distances = np.linalg.norm(candidates-obs.pose, axis=1)
        travel = distances / max(float(distances.max()), 1.0)
        gains = np.asarray([self.information_gain.calculate(obs.combined_obs_map, point)
                            for point in candidates])
        gains /= max(float(gains.max()), 1.0)
        redundancy = np.asarray([self._redundancy(obs, point, collect_opts)
                                 for point in candidates])
        relay = np.clip(np.linalg.norm(candidates-obs.base_pose, axis=1)
                        / (collect_opts.comm_range*collect_opts.pixel_per_meter), 0, 1)
        w = self.weights_for(obs, collect_opts)
        utility = (w.information_gain*gains - w.travel_cost*travel
                   - w.redundancy*redundancy - w.relay_risk*relay)
        costs = inflate_map(obs.combined_obs_map)
        # La validación A* se reserva para los mejores candidatos.
        for index in np.argsort(-utility)[:40]:
            path = astar(costs, obs.pose, candidates[index])
            if path is not None:
                return candidates[index]
        return None

    def weights_for(self, obs: Observation, collect_opts) -> UtilityWeights:
        return self.weights

    @staticmethod
    def _redundancy(obs, point, collect_opts) -> float:
        distances = []
        for values in (*obs.pose_lists_of_others.values(), *obs.intents_of_others.values()):
            if values is not None and len(values):
                distances.append(float(np.linalg.norm(np.asarray(values)-point, axis=1).min()))
        if not distances:
            return 0.0
        scale = max(collect_opts.other_intent_threshold*collect_opts.pixel_per_meter, 1)
        return float(np.clip(1-min(distances)/scale, 0, 1))
