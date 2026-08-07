"""Asignación del objetivo a la frontera más cercana."""

from __future__ import annotations

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotState
from Logic.Robot.Track import CoordinateMatrix

from ..Frontiers import Frontiers


class NearestFrontier(Frontiers):
    def cluster_frontiers(
        self, frontiers: CoordinateMatrix
    ) -> CoordinateMatrix:
        """Retorna fronteras crudas hasta disponer de un clusterizador."""
        return [coordinate.copy() for coordinate in frontiers]

    def assign_goal(
        self,
        belief_map: BeliefMap,
        robot_state: RobotState,
    ) -> list[float]:
        frontiers = self.get_frontiers(belief_map)
        if not frontiers:
            raise ValueError("No existen fronteras disponibles para asignar")
        return min(
            frontiers,
            key=lambda coordinate: (
                (coordinate[0] - robot_state.x) ** 2
                + (coordinate[1] - robot_state.y) ** 2
            ),
        ).copy()


__all__ = ["NearestFrontier"]
