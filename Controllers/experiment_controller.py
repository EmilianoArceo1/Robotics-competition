"""Caso de uso para ejecutar, registrar y persistir experimentos."""

from __future__ import annotations

from random import Random

from Logic.Experiments import (
    ExperimentConfiguration,
    ExperimentExporter,
    ExperimentManager,
    ExperimentRepository,
    ExperimentResult,
)
from Logic.Exploration import ExplorationMetricsSnapshot, ExplorationSnapshot
from Logic.Navigation import NavigationSnapshot


class ExperimentController:
    def __init__(
        self,
        repository: ExperimentRepository | None = None,
        exporter: ExperimentExporter | None = None,
    ) -> None:
        self.manager = ExperimentManager()
        self.repository = repository
        self.exporter = exporter
        self.rng = Random(0)

    def start(self, configuration: ExperimentConfiguration) -> None:
        self.rng = Random(configuration.seed)
        self.manager.start(configuration)

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
        self.manager.record(
            metrics=metrics,
            pose=pose,
            navigation=navigation,
            exploration=exploration,
            safety_active=safety_active,
            collision_rejected=collision_rejected,
        )

    def finish(
        self,
        metrics: ExplorationMetricsSnapshot,
        *,
        outcome: str | None = None,
        reason: str | None = None,
    ) -> ExperimentResult:
        return self.manager.finish(metrics, outcome=outcome, reason=reason)

    @property
    def result(self) -> ExperimentResult | None:
        return self.manager.result

    def save_json(self, destination: str) -> None:
        if self.result is None:
            raise RuntimeError("El experimento aún no ha terminado")
        if self.repository is None:
            raise RuntimeError("No se configuró un repositorio de experimentos")
        self.repository.save(self.result, destination)

    def load_json(self, source: str) -> ExperimentResult:
        if self.repository is None:
            raise RuntimeError("No se configuró un repositorio de experimentos")
        return self.repository.load(source)

    def export_csv(self, destination: str) -> None:
        if self.result is None:
            raise RuntimeError("El experimento aún no ha terminado")
        if self.exporter is None:
            raise RuntimeError("No se configuró un exportador de experimentos")
        self.exporter.export(self.result, destination)


__all__ = ["ExperimentController"]
