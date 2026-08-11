from types import SimpleNamespace

import numpy as np

from Logic.Competition.environment import Observation
from Logic.Competition.return_policies import (DeadlineReturnPolicy,
    EfficientPeriodicReturnPolicy, PayloadAdaptiveReturnPolicy,
    GainSweepReturnPolicy, HomewardSweepReturnPolicy,
    NearestFrontierReturnPolicy,
    PeriodicReturnPolicy, SelectiveCourierReturnPolicy,
    ValueDensityReturnPolicy)


def observation(payload: int = 0):
    observed = np.zeros((40, 40), dtype=float)
    mask = np.zeros_like(observed, dtype=bool)
    mask.flat[:payload] = True
    return Observation(observed, np.array((25, 25)), mask,
        np.zeros_like(mask), {}, {}, np.array((10, 10)), 0, [])


def options():
    return SimpleNamespace(final_relay=True, relay_period=25,
        relay_trigger="periodic", pixel_per_meter=1, comm_range=15)


def test_periodic_return_matches_configured_period():
    assert PeriodicReturnPolicy().decide(observation(), options(), 25, 100) == "relay"


def test_deadline_return_waits_then_commits_to_base():
    policy = DeadlineReturnPolicy()
    assert policy.decide(observation(), options(), 10, 100) is None
    assert policy.decide(observation(), options(), 90, 100) == "final_relay"


def test_payload_return_reacts_to_unreported_information():
    policy = PayloadAdaptiveReturnPolicy(payload_cells=10, distance_m=5)
    assert policy.decide(observation(12), options(), 10, 100) == "relay"


def test_efficient_periodic_keeps_intermediate_delivery():
    assert EfficientPeriodicReturnPolicy().decide(
        observation(), options(), 25, 100) == "relay"


def test_selective_courier_skips_empty_period():
    policy = SelectiveCourierReturnPolicy(minimum_payload=10)
    assert policy.decide(observation(), options(), 25, 100) is None
    assert policy.decide(observation(12), options(), 25, 100) == "relay"


def test_value_density_reacts_to_profitable_payload():
    policy = ValueDensityReturnPolicy(cells_per_return_step=.1,
                                      minimum_distance_m=1)
    assert policy.decide(observation(12), options(), 10, 100) == "relay"


def test_frontier_return_plans_always_finish_at_base_within_budget():
    obs = observation()
    obs.combined_obs_map[30:, :] = .5
    for policy in (NearestFrontierReturnPolicy(), GainSweepReturnPolicy(),
                   HomewardSweepReturnPolicy()):
        path = policy.plan_return(obs, options(), remaining_steps=20)
        assert path is not None
        assert np.array_equal(path[-1], obs.base_pose)
        assert len(path)-1 <= 60
