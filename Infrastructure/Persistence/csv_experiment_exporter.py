from __future__ import annotations

import csv
from pathlib import Path

from Logic.Experiments import ExperimentResult


class CsvExperimentExporter:
    FIELDS = (
        "experiment", "seed", "path_planner", "objective_assigner",
        "clustering", "safe_tracker", "coordination", "outcome", "elapsed_time",
        "distance_traveled", "coverage", "goals_reached", "failed_goals",
        "replans", "safety_interventions", "rejected_collisions",
    )

    def export(self, result: ExperimentResult, destination: str) -> None:
        path = Path(destination)
        path.parent.mkdir(parents=True, exist_ok=True)
        config, summary = result.configuration, result.summary
        row = {
            "experiment": config.name,
            "seed": config.seed,
            "path_planner": config.algorithms.path_planner,
            "objective_assigner": config.algorithms.objective_assigner,
            "clustering": config.algorithms.clustering,
            "safe_tracker": config.algorithms.safe_tracker,
            "coordination": config.algorithms.coordination,
            **{field: getattr(summary, field) for field in self.FIELDS if hasattr(summary, field)},
        }
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.FIELDS)
            writer.writeheader()
            writer.writerow(row)


__all__ = ["CsvExperimentExporter"]
