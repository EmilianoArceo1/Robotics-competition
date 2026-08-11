"""Entrega seleccionable desde 'CARGAR POLICY .PY'."""
from Logic.Competition import (NextBestViewPolicy,
    PotentialVisibilityInformationGain, UtilityWeights)


class Policy(NextBestViewPolicy):
    def __init__(self):
        super().__init__(
            weights=UtilityWeights(.50, .25, .20, .05),
            information_gain=PotentialVisibilityInformationGain(radius=100, rays=180),
        )
