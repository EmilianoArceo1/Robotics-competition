import pytest

from Logic.Competition import UtilityWeights
from Logic.Competition.experiments import TrialResult, leave_one_environment_out, simplex_weights, summarize


def trial(weights, environment, size, robots, coverage):
    return TrialResult(weights.label, weights.information_gain, weights.travel_cost,
        weights.redundancy, weights.relay_risk, environment, size, robots,
        15, 15, 500, coverage)


def test_coarse_simplex_contains_35_normalized_combinations():
    values = simplex_weights(.25)
    assert len(values) == 35
    assert all(sum((w.information_gain, w.travel_cost, w.redundancy, w.relay_risk)) == 1 for w in values)


def test_weights_reject_non_normalized_values():
    with pytest.raises(ValueError, match="igual a 1"):
        UtilityWeights(.5, .5, .5, 0)


def test_summary_and_leave_one_out_report_robust_metrics():
    a, b = UtilityWeights(1,0,0,0), UtilityWeights(.5,.5,0,0)
    results = [trial(a,"env1","Small",1,.7), trial(a,"env2","Small",3,.8),
               trial(b,"env1","Small",1,.6), trial(b,"env2","Small",3,.9)]
    table = summarize(results)
    assert {"Mean","Median","Worst","StdDev","Single","Multi"}.issubset(table[0])
    folds = leave_one_environment_out(results)
    assert [row["HeldOut"] for row in folds] == ["env1", "env2"]
