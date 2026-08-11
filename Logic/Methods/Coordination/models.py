"""Contratos inmutables compartidos por los métodos de coordinación."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

Coordinate = tuple[float, float]


class CoordinationMode(str, Enum):
    NONE = "none"
    CENTRALIZED = "centralized"
    DECENTRALIZED = "decentralized"


@dataclass(frozen=True, slots=True)
class RobotCoordinationState:
    robot_id: str
    position: Coordinate
    heading: float
    current_goal: Coordinate | None = None
    navigation_state: str = "IDLE"
    available: bool = True


@dataclass(frozen=True, slots=True)
class CoordinationMessage:
    sender_id: str
    kind: str
    goal: Coordinate | None = None
    bid: float | None = None
    recipient_id: str | None = None
    expires_at: float | None = None


@dataclass(frozen=True, slots=True)
class CoordinationContext:
    timestamp: float
    local_robot_id: str
    robots: tuple[RobotCoordinationState, ...]
    candidate_goals: tuple[Coordinate, ...]
    incoming_messages: tuple[CoordinationMessage, ...] = ()
    shared_belief: tuple[tuple[Coordinate, int], ...] | None = None


@dataclass(frozen=True, slots=True)
class GoalAssignment:
    robot_id: str
    goal: Coordinate
    cost: float


@dataclass(frozen=True, slots=True)
class CoordinationDecision:
    mode: CoordinationMode
    assignments: tuple[GoalAssignment, ...]
    outgoing_messages: tuple[CoordinationMessage, ...]
    reason: str

    def goal_for(self, robot_id: str) -> Coordinate | None:
        return next(
            (item.goal for item in self.assignments if item.robot_id == robot_id),
            None,
        )


__all__ = [
    "Coordinate", "CoordinationContext", "CoordinationDecision",
    "CoordinationMessage", "CoordinationMode", "GoalAssignment",
    "RobotCoordinationState",
]
