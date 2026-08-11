"""Mejor combinación proxy de exploración, retorno y handoff."""

from Logic.Competition import BasePolicy
from Logic.Competition.advanced_policies import TrajectoryDiversifiedNearestPolicy
from Logic.Competition.handoff_policies import LinkQualityHandoffPolicy
from Logic.Competition.return_policies import SelectiveCourierReturnPolicy


class Policy(BasePolicy):
    """Policy autocontenida compatible con el contrato de competición."""

    def __init__(self) -> None:
        self.exploration = TrajectoryDiversifiedNearestPolicy()
        self.returning = SelectiveCourierReturnPolicy()
        self.handoff = LinkQualityHandoffPolicy()

    def decide(self, obs, collect_opts):
        return self.exploration.decide(obs, collect_opts)

    def should_relay(self, obs, collect_opts, t, max_steps):
        return self.returning.decide(obs, collect_opts, t, max_steps)

    def decide_relay_handoff(self, obs, collect_opts):
        return self.handoff.decide(obs, collect_opts)
