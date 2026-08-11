"""Asignación del objetivo a la frontera más cercana."""

from __future__ import annotations

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotState
from Logic.Robot.Track import CoordinateMatrix

from ..Frontiers import Frontiers


class NearestFrontier(Frontiers):
    def rank_goals(
        self,
        belief_map: BeliefMap,
        robot_state: RobotState,
    ) -> CoordinateMatrix:
        """Devuelve candidatos en el orden definido por esta estrategia."""
        frontiers = self.get_frontiers(belief_map)
        return sorted(
            frontiers,
            key=lambda coordinate: (
                (coordinate[0] - robot_state.x) ** 2
                + (coordinate[1] - robot_state.y) ** 2,
                coordinate[1],
                coordinate[0],
            ),
        )

    def assign_goal(
        self,
        belief_map: BeliefMap,
        robot_state: RobotState,
    ) -> list[float]:
        frontiers = self.rank_goals(belief_map, robot_state)
        if not frontiers:
            raise ValueError("No existen fronteras disponibles para asignar")
        return frontiers[0].copy()


__all__ = ["NearestFrontier"]
