import numpy as np

from Logic.Competition import (CircularUnknownInformationGain,
    FrontierDensityInformationGain, InformationGainMethod)


def test_information_gain_methods_share_abstract_contract():
    observed = np.full((21, 21), .5)
    observed[10, 5:11] = 0
    methods = (CircularUnknownInformationGain(5), FrontierDensityInformationGain(5))
    assert all(isinstance(method, InformationGainMethod) for method in methods)
    assert all(method.calculate(observed, np.array((10,10))) >= 0 for method in methods)
