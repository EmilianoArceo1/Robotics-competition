"""Utilidades públicas compatibles con las policies de la competición."""
from math import ceil
import numpy as np

from Logic.Competition.environment import astar, get_frontiers, inflate_map


def crowding_avoidance_penalty(candidate_centers, scores, pose_lists_of_others,
                               intents_of_others, collect_opts):
    result = np.asarray(scores, dtype=float).copy()
    for index, center in enumerate(np.asarray(candidate_centers)):
        trajectories = [np.linalg.norm(np.asarray(value) - center, axis=1).min()
                        for value in pose_lists_of_others.values() if value is not None and len(value)]
        if trajectories and min(trajectories) < collect_opts.other_traj_threshold * collect_opts.pixel_per_meter:
            result[index] -= 1_000_000
        intents = [np.linalg.norm(np.asarray(value) - center, axis=1).min()
                   for value in intents_of_others.values() if value is not None and len(value)]
        if intents and min(intents) < collect_opts.other_intent_threshold * collect_opts.pixel_per_meter:
            result[index] -= 100_000
    return result


def estimate_time_for_path(path, plan_ind_to_use=3):
    return ceil((len(path) - 1) / plan_ind_to_use)


def default_should_relay(obs, collect_opts, t, max_steps):
    from Logic.Competition.environment import BasePolicy
    return BasePolicy().should_relay(obs, collect_opts, t, max_steps)


def default_relay_handoff(obs, collect_opts):
    from Logic.Competition.environment import BasePolicy
    return BasePolicy().decide_relay_handoff(obs, collect_opts)


__all__ = ["get_frontiers", "inflate_map", "crowding_avoidance_penalty",
           "estimate_time_for_path", "default_should_relay", "default_relay_handoff"]
