"""Policy de ejemplo compatible con el formato de entrega oficial."""
import numpy as np
import pyastar2d

from base_policy import BasePolicy
from policy_utils import crowding_avoidance_penalty, get_frontiers, inflate_map


class Policy(BasePolicy):
    def decide(self, obs, collect_opts):
        frontiers = get_frontiers(obs.combined_obs_map)
        if not frontiers:
            return None
        frontiers = np.asarray(frontiers)
        scores = -np.linalg.norm(frontiers - obs.pose, axis=1)
        scores = crowding_avoidance_penalty(
            frontiers, scores, obs.pose_lists_of_others,
            obs.intents_of_others, collect_opts,
        )
        costs = inflate_map(obs.combined_obs_map)
        for index in np.argsort(-scores):
            goal = frontiers[index]
            if pyastar2d.astar_path(costs, obs.pose, goal,
                                    allow_diagonal=False) is not None:
                return goal
        return None
