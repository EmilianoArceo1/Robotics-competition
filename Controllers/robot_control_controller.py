"""Implementación concreta de Control para conectar estrategias seleccionables."""

from __future__ import annotations

from typing import Protocol

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Control import Control
from Logic.Robot.Physic import RobotState


class ObjectiveAssigner(Protocol):
    def assign_goal(
        self, belief_map: BeliefMap, robot_state: RobotState
    ) -> list[float]: ...


class RobotControlController(Control):
    def __init__(
        self,
        objective_assigner: ObjectiveAssigner,
        **control_options: object,
    ) -> None:
        super().__init__(**control_options)
        self.objective_assigner = objective_assigner

    def assign_goal(
        self,
        belief_map: BeliefMap,
        robot_state: RobotState,
    ) -> list[float]:
        return self.objective_assigner.assign_goal(belief_map, robot_state)
