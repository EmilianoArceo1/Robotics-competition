"""Ciclo de vida de objetivos de exploración, independiente de la UI."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Iterable, Sequence

Coordinate = tuple[float, float]


class ExplorationState(str, Enum):
    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True, slots=True)
class ExplorationDecision:
    goal: Coordinate | None
    state: ExplorationState
    reason: str
    candidate_count: int
    rejected_count: int


@dataclass(frozen=True, slots=True)
class ExplorationSnapshot:
    state: ExplorationState
    current_goal: Coordinate | None
    visited_goals: int
    failed_goals: int
    reason: str


class ExplorationManager:
    """Selecciona entre candidatos ya ordenados y registra sus resultados.

    La estrategia de asignación conserva la responsabilidad de producir y
    ordenar candidatos. Este servicio sólo administra su ciclo de vida.
    """

    def __init__(self, *, failed_goal_cooldown: int = 120) -> None:
        if failed_goal_cooldown <= 0:
            raise ValueError("failed_goal_cooldown debe ser positivo")
        self.failed_goal_cooldown = int(failed_goal_cooldown)
        self.reset()

    def reset(self) -> None:
        self.state = ExplorationState.IDLE
        self.reason = "waiting for observations"
        self.current_goal: Coordinate | None = None
        self._visited: set[Coordinate] = set()
        self._failed: dict[Coordinate, int] = {}

    @staticmethod
    def _coordinate(value: Sequence[float]) -> Coordinate:
        if len(value) != 2:
            raise ValueError("Cada objetivo debe contener [x, y]")
        coordinate = round(float(value[0]), 9), round(float(value[1]), 9)
        if not all(isfinite(component) for component in coordinate):
            raise ValueError("Las coordenadas del objetivo deben ser finitas")
        return coordinate

    def begin_tick(self) -> None:
        expired = []
        for goal, ticks in self._failed.items():
            if ticks <= 1:
                expired.append(goal)
            else:
                self._failed[goal] = ticks - 1
        for goal in expired:
            del self._failed[goal]

    def select_goal(
        self, ordered_candidates: Iterable[Sequence[float]]
    ) -> ExplorationDecision:
        candidates = tuple(
            dict.fromkeys(self._coordinate(value) for value in ordered_candidates)
        )
        rejected = 0
        for goal in candidates:
            if goal in self._visited or goal in self._failed:
                rejected += 1
                continue
            self.current_goal = goal
            self.state = ExplorationState.EXPLORING
            self.reason = "goal selected"
            return ExplorationDecision(
                goal, self.state, self.reason, len(candidates), rejected
            )

        self.current_goal = None
        if not candidates or all(goal in self._visited for goal in candidates):
            self.state = ExplorationState.COMPLETE
            self.reason = "no unexplored goals remain"
        else:
            self.state = ExplorationState.EXPLORING
            self.reason = "goals temporarily unavailable"
        return ExplorationDecision(
            None, self.state, self.reason, len(candidates), rejected
        )

    def mark_reached(self, goal: Sequence[float] | None = None) -> None:
        selected = self.current_goal if goal is None else self._coordinate(goal)
        if selected is not None:
            self._visited.add(selected)
            self._failed.pop(selected, None)
        self.current_goal = None
        self.reason = "goal explored"

    def mark_failed(self, goal: Sequence[float] | None = None) -> None:
        selected = self.current_goal if goal is None else self._coordinate(goal)
        if selected is not None:
            self._failed[selected] = self.failed_goal_cooldown
        self.current_goal = None
        self.state = ExplorationState.EXPLORING
        self.reason = "goal temporarily rejected"

    @property
    def snapshot(self) -> ExplorationSnapshot:
        return ExplorationSnapshot(
            self.state,
            self.current_goal,
            len(self._visited),
            len(self._failed),
            self.reason,
        )


__all__ = [
    "ExplorationDecision",
    "ExplorationManager",
    "ExplorationSnapshot",
    "ExplorationState",
]
