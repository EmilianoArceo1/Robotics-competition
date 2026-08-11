"""Adaptación de la simulación al contrato puro de coordinación."""

from __future__ import annotations

from dataclasses import dataclass

from Controllers.robot_control_controller import RobotControlController
from Logic.Methods.Coordination import (
    CoordinationContext, CoordinationDecision, CoordinationTransport,
    Coordinator, RobotCoordinationState,
)


@dataclass(frozen=True, slots=True)
class CoordinationResult:
    ordered_goals: tuple[tuple[float, float], ...]
    decision: CoordinationDecision
    error: str | None = None


class CoordinationRuntime:
    def __init__(
        self,
        coordinator: Coordinator,
        *,
        robot_id: str = "robot-1",
        transport: CoordinationTransport | None = None,
    ) -> None:
        self.coordinator = coordinator
        self.robot_id = robot_id
        self.transport = transport
        self.last_decision: CoordinationDecision | None = None

    def run(
        self,
        control: RobotControlController,
        *,
        timestamp: float,
        navigation_state: str,
        current_goal: tuple[float, float] | None,
        fleet_members=None,
    ) -> CoordinationResult:
        state = control.physics.state
        try:
            rank_goals = getattr(control.objective_assigner, "rank_goals", None)
            if callable(rank_goals):
                candidates = tuple(
                    (float(goal[0]), float(goal[1]))
                    for goal in rank_goals(control.belief_map, state)
                )
            else:
                goal = control.assign_goal(control.belief_map, state)
                candidates = ((float(goal[0]), float(goal[1])),)
        except Exception as error:
            decision = CoordinationDecision(
                self.coordinator.mode, (), (), f"coordination input failed: {error}"
            )
            self.last_decision = decision
            return CoordinationResult((), decision, str(error))

        incoming = (
            self.transport.receive(self.robot_id, timestamp)
            if self.transport is not None else ()
        )
        belief = tuple(
            ((float(cell[0][0]), float(cell[0][1])), int(cell[1]))
            for cell in control.belief_map.matrix
        )
        robots = (
            tuple(
                RobotCoordinationState(
                    member.robot_id,
                    (member.physics.state.x, member.physics.state.y),
                    member.physics.state.theta,
                    current_goal if member.robot_id == self.robot_id else None,
                    navigation_state if member.robot_id == self.robot_id else "PASSIVE",
                    member.controllable,
                )
                for member in fleet_members
            )
            if fleet_members is not None
            else (
                RobotCoordinationState(
                    self.robot_id, (state.x, state.y), state.theta,
                    current_goal, navigation_state, True,
                ),
            )
        )
        context = CoordinationContext(
            timestamp,
            self.robot_id,
            robots,
            candidates,
            incoming,
            belief,
        )
        try:
            decision = self.coordinator.coordinate(context)
        except Exception as error:
            decision = CoordinationDecision(
                self.coordinator.mode, (), (), f"coordination failed: {error}"
            )
            self.last_decision = decision
            return CoordinationResult((), decision, str(error))
        if self.transport is not None:
            for message in decision.outgoing_messages:
                self.transport.publish(message)
        assigned = decision.goal_for(self.robot_id)
        ordered = (
            candidates if assigned is None
            else (assigned, *(goal for goal in candidates if goal != assigned))
        )
        self.last_decision = decision
        return CoordinationResult(tuple(ordered), decision)


__all__ = ["CoordinationResult", "CoordinationRuntime"]
