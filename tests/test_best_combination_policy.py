from Logic.Competition import BasePolicy
from Logic.Competition.environment import load_policy


def test_best_combination_loads_official_three_method_contract():
    policy = load_policy("Policies/competition_best_combination.py")
    assert isinstance(policy, BasePolicy)
    assert callable(policy.decide)
    assert callable(policy.should_relay)
    assert callable(policy.decide_relay_handoff)
