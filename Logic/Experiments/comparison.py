"""Comparación determinista de resultados sin conocer su persistencia."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .models import ExperimentResult


@dataclass(frozen=True, slots=True)
class ComparisonEntry:
    rank: int
    experiment: str
    seed: int
    algorithm: str
    coverage: float
    elapsed_time: float
    distance_traveled: float
    rejected_collisions: int
    outcome: str


def compare_experiments(
    results: Iterable[ExperimentResult],
) -> tuple[ComparisonEntry, ...]:
    ordered = sorted(
        results,
        key=lambda result: (
            -result.summary.coverage,
            result.summary.rejected_collisions,
            result.summary.elapsed_time,
            result.summary.distance_traveled,
            result.configuration.name,
        ),
    )
    return tuple(
        ComparisonEntry(
            index,
            result.configuration.name,
            result.configuration.seed,
            result.configuration.algorithms.path_planner,
            result.summary.coverage,
            result.summary.elapsed_time,
            result.summary.distance_traveled,
            result.summary.rejected_collisions,
            result.summary.outcome,
        )
        for index, result in enumerate(ordered, start=1)
    )


__all__ = ["ComparisonEntry", "compare_experiments"]
