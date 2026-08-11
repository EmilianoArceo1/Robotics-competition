"""Registro en memoria de una ejecución, sin dependencias de infraestructura."""

from __future__ import annotations

from Logic.Exploration import ExplorationMetricsSnapshot, ExplorationSnapshot
from Logic.Navigation import NavigationSnapshot

from .models import (
    ExperimentConfiguration,
    ExperimentResult,
    ExperimentSample,
    ExperimentSummary,
)


class ExperimentManager:
    def __init__(self) -> None:
        self._configuration: ExperimentConfiguration | None = None
        self._samples: list[ExperimentSample] = []
        self._result: ExperimentResult | None = None

    @property
    def active(self) -> bool:
        return self._configuration is not None and self._result is None

    def start(self, configuration: ExperimentConfiguration) -> None:
        if not configuration.name.strip():
            raise ValueError("El experimento debe tener un nombre")
        if self.active:
            raise RuntimeError("El experimento activo debe finalizar primero")
        self._configuration = configuration
        self._samples = []
        self._result = None

    def record(
        self,
        *,
        metrics: ExplorationMetricsSnapshot,
        pose: tuple[float, float, float],
        navigation: NavigationSnapshot,
        exploration: ExplorationSnapshot,
        safety_active: bool,
        collision_rejected: bool,
    ) -> None:
        if not self.active:
            return
        self._samples.append(
            ExperimentSample(
                metrics.elapsed_time,
                float(pose[0]),
                float(pose[1]),
                float(pose[2]),
                metrics.coverage,
                navigation.state.value,
                exploration.state.value,
                navigation.current_goal,
                bool(safety_active),
                bool(collision_rejected),
            )
        )

    def finish(
        self,
        metrics: ExplorationMetricsSnapshot,
        *,
        outcome: str | None = None,
        reason: str | None = None,
    ) -> ExperimentResult:
        if self._configuration is None:
            raise RuntimeError("No hay un experimento iniciado")
        summary = ExperimentSummary(
            outcome or metrics.outcome.value,
            reason or metrics.reason,
            metrics.elapsed_time,
            metrics.distance_traveled,
            metrics.coverage,
            metrics.known_cells,
            metrics.total_cells,
            metrics.goals_reached,
            metrics.failed_goals,
            metrics.replans,
            metrics.safety_interventions,
            metrics.rejected_collisions,
        )
        self._result = ExperimentResult(
            self._configuration, summary, tuple(self._samples)
        )
        return self._result

    @property
    def result(self) -> ExperimentResult | None:
        return self._result


__all__ = ["ExperimentManager"]
