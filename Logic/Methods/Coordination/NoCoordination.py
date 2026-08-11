from __future__ import annotations

from math import hypot

from .Coordinator import Coordinator
from .models import (
    CoordinationContext, CoordinationDecision, CoordinationMode, GoalAssignment,
)


class NoCoordination(Coordinator):
    @property
    def mode(self) -> CoordinationMode:
        return CoordinationMode.NONE

    def coordinate(self, context: CoordinationContext) -> CoordinationDecision:
        robot = next(
            (item for item in context.robots if item.robot_id == context.local_robot_id),
            None,
        )
        if robot is None or not robot.available or not context.candidate_goals:
            return CoordinationDecision(self.mode, (), (), "no local assignment")
        goal = context.candidate_goals[0]
        cost = hypot(goal[0] - robot.position[0], goal[1] - robot.position[1])
        return CoordinationDecision(
            self.mode,
            (GoalAssignment(robot.robot_id, goal, cost),),
            (),
            "local objective preserved",
        )


__all__ = ["NoCoordination"]
