import numpy as np

from Logic.Competition import (FrontierOcclusionViewpointGenerator,
    NextBestViewPolicy, PotentialVisibilityInformationGain)
from Logic.Competition.environment import Observation


def test_generator_returns_only_known_free_viewpoints():
    observed = np.full((80, 80), .5)
    observed[20:60, 10:40] = 0
    observed[20:55, 40] = 1
    generator = FrontierOcclusionViewpointGenerator(maximum_candidates=50)
    candidates = generator.generate(observed, np.array((40, 20)))
    assert len(candidates)
    assert all(observed[tuple(point)] == 0 for point in candidates)


def test_raycast_gain_respects_known_occlusion():
    observed = np.full((41, 41), .5)
    observed[20, 5:15] = 0
    open_gain = PotentialVisibilityInformationGain(20, 64).calculate(observed, np.array((20,10)))
    observed[:, 15] = 1
    blocked_gain = PotentialVisibilityInformationGain(20, 64).calculate(observed, np.array((20,10)))
    assert blocked_gain < open_gain
