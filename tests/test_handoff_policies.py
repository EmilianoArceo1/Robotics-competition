from types import SimpleNamespace

import numpy as np

from Logic.Competition.environment import Observation
from Logic.Competition.handoff_policies import (LinkQualityHandoffPolicy,
    PayloadProgressHandoffPolicy, ReturningCourierHandoffPolicy,
    TimeSavingHandoffPolicy)


def observation(payload=400, peer=(15, 15)):
    mask = np.zeros((30, 30), dtype=bool)
    mask.flat[:payload] = True
    trajectory = np.asarray(((16, 16), peer), dtype=float)
    return Observation(np.zeros((30, 30)), np.array((25, 25)), mask,
        np.zeros_like(mask), {"robot2": trajectory}, {}, np.array((5, 5)), 0, [2])


def options():
    return SimpleNamespace(pixel_per_meter=1, comm_range=30)


def test_payload_progress_selects_useful_receiver():
    assert PayloadProgressHandoffPolicy().decide(observation(), options()) == 2


def test_payload_progress_rejects_small_delivery():
    assert PayloadProgressHandoffPolicy().decide(observation(20), options()) is None


def test_payload_progress_rejects_receiver_without_progress():
    assert PayloadProgressHandoffPolicy().decide(
        observation(peer=(26, 26)), options()) is None


def test_advanced_handoffs_select_progressing_courier():
    obs = observation()
    for policy in (TimeSavingHandoffPolicy(), ReturningCourierHandoffPolicy(),
                   LinkQualityHandoffPolicy()):
        assert policy.decide(obs, options()) == 2
