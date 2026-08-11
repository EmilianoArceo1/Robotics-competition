"""Máquina de estados y memoria de recuperación de navegación."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import hypot


class NavigationState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    FOLLOWING = "FOLLOWING"
    GOAL_REACHED = "GOAL_REACHED"
    BLOCKED = "BLOCKED"
    RECOVERING = "RECOVERING"
    EXHAUSTED = "EXHAUSTED"


@dataclass(frozen=True, slots=True)
class NavigationSnapshot:
    state: NavigationState
    reason: str
    replan_count: int
    failed_goal_count: int
    blocked_ticks: int
    current_goal: tuple[float, float] | None
    last_failed_goal: tuple[float, float] | None


class NavigationSupervisor:
    def __init__(
        self,
        *,
        blocked_tick_limit: int = 30,
        failed_goal_cooldown: int = 120,
        progress_epsilon: float = 1e-4,
    ) -> None:
        self.blocked_tick_limit = int(blocked_tick_limit)
        self.failed_goal_cooldown = int(failed_goal_cooldown)
        self.progress_epsilon = float(progress_epsilon)
        self.reset()

    def reset(self) -> None:
        self.state = NavigationState.IDLE
        self.reason = "waiting for route"
        self.replan_count = 0
        self.blocked_ticks = 0
        self.current_goal: tuple[float, float] | None = None
        self.last_failed_goal: tuple[float, float] | None = None
        self._failed_goals: dict[tuple[float, float], int] = {}
        self._last_position: tuple[float, float] | None = None

    def begin_tick(self) -> None:
        expired: list[tuple[float, float]] = []
        for goal, remaining in self._failed_goals.items():
            if remaining <= 1:
                expired.append(goal)
            else:
                self._failed_goals[goal] = remaining - 1
        for goal in expired:
            del self._failed_goals[goal]

    @staticmethod
    def _goal_key(goal: tuple[float, float]) -> tuple[float, float]:
        return round(float(goal[0]), 9), round(float(goal[1]), 9)

    def goal_available(self, goal: tuple[float, float]) -> bool:
        return self._goal_key(goal) not in self._failed_goals

    def planning(self) -> None:
        self.state = NavigationState.PLANNING
        self.reason = "planning route"

    def route_accepted(self, goal: tuple[float, float]) -> None:
        self.current_goal = self._goal_key(goal)
        self.state = NavigationState.FOLLOWING
        self.reason = "following validated route"
        self.blocked_ticks = 0

    def route_invalidated(self, reason: str) -> None:
        self.state = NavigationState.RECOVERING
        self.reason = reason
        self.replan_count += 1

    def planning_failed(self, goal: tuple[float, float], reason: str) -> None:
        key = self._goal_key(goal)
        self._failed_goals[key] = self.failed_goal_cooldown
        self.last_failed_goal = key
        self.current_goal = None
        self.state = NavigationState.RECOVERING
        self.reason = reason
        self.replan_count += 1

    def exhausted(self, reason: str = "no reachable goals") -> None:
        self.state = NavigationState.EXHAUSTED
        self.reason = reason
        self.current_goal = None

    def goal_reached(self) -> None:
        self.state = NavigationState.GOAL_REACHED
        self.reason = "goal reached"
        self.current_goal = None
        self.blocked_ticks = 0

    def observe_motion(
        self,
        position: tuple[float, float],
        *,
        commanded: bool,
        safety_active: bool,
        collision: bool,
    ) -> bool:
        if self._last_position is None:
            self._last_position = position
            return False
        progress = hypot(
            position[0] - self._last_position[0],
            position[1] - self._last_position[1],
        )
        self._last_position = position
        obstructed = commanded and (
            progress <= self.progress_epsilon
            and (safety_active or collision)
        )
        self.blocked_ticks = self.blocked_ticks + 1 if obstructed else 0
        if self.blocked_ticks >= self.blocked_tick_limit:
            self.state = NavigationState.BLOCKED
            self.reason = "robot blocked by safety constraints"
            self.replan_count += 1
            self.blocked_ticks = 0
            return True
        return False

    @property
    def snapshot(self) -> NavigationSnapshot:
        return NavigationSnapshot(
            self.state,
            self.reason,
            self.replan_count,
            len(self._failed_goals),
            self.blocked_ticks,
            self.current_goal,
            self.last_failed_goal,
        )
