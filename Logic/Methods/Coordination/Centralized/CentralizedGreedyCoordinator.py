from __future__ import annotations

from math import hypot

from ..Coordinator import Coordinator
from ..models import (
    CoordinationContext, CoordinationDecision, CoordinationMode, GoalAssignment,
)


class CentralizedGreedyCoordinator(Coordinator):
    @property
    def mode(self) -> CoordinationMode:
        return CoordinationMode.CENTRALIZED

    def coordinate(self, context: CoordinationContext) -> CoordinationDecision:
        robots = tuple(robot for robot in context.robots if robot.available)
        pairs = sorted(
            (
                (
                    hypot(goal[0] - robot.position[0], goal[1] - robot.position[1]),
                    robot.robot_id,
                    goal,
                )
                for robot in robots
                for goal in context.candidate_goals
            ),
            key=lambda item: (item[0], item[1], item[2][1], item[2][0]),
        )
        assigned_robots: set[str] = set()
        assigned_goals: set[tuple[float, float]] = set()
        assignments = []
        for cost, robot_id, goal in pairs:
            if robot_id in assigned_robots or goal in assigned_goals:
                continue
            assigned_robots.add(robot_id)
            assigned_goals.add(goal)
            assignments.append(GoalAssignment(robot_id, goal, cost))
        return CoordinationDecision(
            self.mode, tuple(assignments), (),
            "globally unique greedy assignments",
        )


__all__ = ["CentralizedGreedyCoordinator"]
