"""Métricas y terminación formal de una ejecución de exploración."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import hypot
from typing import Iterable, Sequence

from Logic.Navigation import NavigationSnapshot, NavigationState
from .ExplorationManager import ExplorationSnapshot, ExplorationState


class ExplorationOutcome(str, Enum):
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    PARTIALLY_COMPLETED = "PARTIALLY_COMPLETED"
    STALLED = "STALLED"
    NO_REACHABLE_FRONTIERS = "NO_REACHABLE_FRONTIERS"
    ABORTED = "ABORTED"


@dataclass(frozen=True, slots=True)
class ExplorationMetricsSnapshot:
    outcome: ExplorationOutcome
    reason: str
    elapsed_time: float
    distance_traveled: float
    total_cells: int
    known_cells: int
    free_cells: int
    occupied_cells: int
    unknown_cells: int
    coverage: float
    coverage_per_second: float
    goals_reached: int
    failed_goals: int
    replans: int
    safety_interventions: int
    rejected_collisions: int


class ExplorationMetrics:
    def __init__(
        self,
        *,
        completion_threshold: float = 0.99,
        stalled_tick_limit: int = 120,
    ) -> None:
        if not 0.0 < completion_threshold <= 1.0:
            raise ValueError("completion_threshold debe estar entre 0 y 1")
        if stalled_tick_limit <= 0:
            raise ValueError("stalled_tick_limit debe ser positivo")
        self.completion_threshold = float(completion_threshold)
        self.stalled_tick_limit = int(stalled_tick_limit)
        self.reset()

    def reset(self) -> None:
        self.elapsed_time = 0.0
        self.distance_traveled = 0.0
        self.total_cells = 0
        self.known_cells = 0
        self.free_cells = 0
        self.occupied_cells = 0
        self.unknown_cells = 0
        self.safety_interventions = 0
        self.rejected_collisions = 0
        self._last_position: tuple[float, float] | None = None
        self._safety_was_active = False
        self._stalled_ticks = 0
        self._outcome = ExplorationOutcome.RUNNING
        self._reason = "exploration in progress"
        self._goals_reached = 0
        self._failed_goals = 0
        self._replans = 0

    def record_tick(
        self,
        *,
        dt: float,
        position: tuple[float, float],
        belief_matrix: Iterable[Sequence[object]],
        total_cells: int,
        exploration: ExplorationSnapshot,
        navigation: NavigationSnapshot,
        safety_active: bool,
        collision_rejected: bool,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("dt debe ser positivo")
        self.elapsed_time += float(dt)
        if self._last_position is not None:
            self.distance_traveled += hypot(
                position[0] - self._last_position[0],
                position[1] - self._last_position[1],
            )
        self._last_position = float(position[0]), float(position[1])

        values = [int(cell[1]) for cell in belief_matrix]
        self.free_cells = values.count(0)
        self.occupied_cells = values.count(1)
        self.known_cells = self.free_cells + self.occupied_cells
        self.total_cells = max(0, int(total_cells))
        self.unknown_cells = max(0, self.total_cells - self.known_cells)
        if safety_active and not self._safety_was_active:
            self.safety_interventions += 1
        self._safety_was_active = bool(safety_active)
        if collision_rejected:
            self.rejected_collisions += 1

        self._goals_reached = exploration.visited_goals
        self._failed_goals = exploration.failed_goals
        self._replans = navigation.replan_count
        self._update_outcome(exploration, navigation)

    def _update_outcome(
        self,
        exploration: ExplorationSnapshot,
        navigation: NavigationSnapshot,
    ) -> None:
        if self._outcome == ExplorationOutcome.ABORTED:
            return
        if exploration.state == ExplorationState.COMPLETE:
            if self.coverage >= self.completion_threshold:
                self._outcome = ExplorationOutcome.COMPLETED
                self._reason = "coverage threshold reached"
            else:
                self._outcome = ExplorationOutcome.PARTIALLY_COMPLETED
                self._reason = "no goals remain before full coverage"
            return

        stalled = navigation.state in {
            NavigationState.BLOCKED,
            NavigationState.EXHAUSTED,
        }
        self._stalled_ticks = self._stalled_ticks + 1 if stalled else 0
        if self._stalled_ticks >= self.stalled_tick_limit:
            if exploration.failed_goals:
                self._outcome = ExplorationOutcome.NO_REACHABLE_FRONTIERS
                self._reason = "frontier goals remained unreachable"
            else:
                self._outcome = ExplorationOutcome.STALLED
                self._reason = "exploration made no navigational progress"
        else:
            self._outcome = ExplorationOutcome.RUNNING
            self._reason = "exploration in progress"

    def abort(self, reason: str = "aborted by user") -> None:
        self._outcome = ExplorationOutcome.ABORTED
        self._reason = str(reason)

    @property
    def coverage(self) -> float:
        return self.known_cells / self.total_cells if self.total_cells else 0.0

    @property
    def snapshot(self) -> ExplorationMetricsSnapshot:
        return ExplorationMetricsSnapshot(
            self._outcome,
            self._reason,
            self.elapsed_time,
            self.distance_traveled,
            self.total_cells,
            self.known_cells,
            self.free_cells,
            self.occupied_cells,
            self.unknown_cells,
            self.coverage,
            self.coverage / self.elapsed_time if self.elapsed_time else 0.0,
            self._goals_reached,
            self._failed_goals,
            self._replans,
            self.safety_interventions,
            self.rejected_collisions,
        )


__all__ = [
    "ExplorationMetrics",
    "ExplorationMetricsSnapshot",
    "ExplorationOutcome",
]
