from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from Infrastructure.Persistence import CsvExperimentExporter, JsonExperimentRepository
from Logic.Experiments import (
    AlgorithmConfiguration, ExperimentConfiguration, ExperimentResult,
    ExperimentSample, ExperimentSummary, MapConfiguration,
    RobotConfiguration, SensorConfiguration,
    compare_experiments,
)


def result_fixture() -> ExperimentResult:
    return ExperimentResult(
        ExperimentConfiguration(
            "baseline", 42,
            AlgorithmConfiguration("A*", "Nearest", "Connected", "CBF"),
            SensorConfiguration("lidar", 270.0, 8.0, 0.5),
            RobotConfiguration(0.5, 0.35, 0.2),
            MapConfiguration(((2.0, 0.0),), 1.0, (0.0, 0.0)),
        ),
        ExperimentSummary(
            "COMPLETED", "done", 10.0, 4.0, 1.0, 20, 20,
            3, 1, 2, 4, 2,
        ),
        (ExperimentSample(
            0.1, 0.0, 0.0, 0.0, 0.2, "FOLLOWING", "EXPLORING",
            (1.0, 0.0), False, False,
        ),),
    )


class ExperimentPersistenceTests(unittest.TestCase):
    def test_json_round_trip_preserves_domain_result(self) -> None:
        repository = JsonExperimentRepository()
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "experiment.json")
            expected = result_fixture()
            repository.save(expected, path)
            self.assertEqual(repository.load(path), expected)

    def test_csv_exports_comparable_summary_row(self) -> None:
        exporter = CsvExperimentExporter()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "experiment.csv"
            exporter.export(result_fixture(), str(path))
            with path.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["seed"], "42")
            self.assertEqual(rows[0]["path_planner"], "A*")
            self.assertEqual(rows[0]["outcome"], "COMPLETED")

    def test_comparison_prioritizes_coverage_then_safety(self) -> None:
        baseline = result_fixture()
        safer = ExperimentResult(
            ExperimentConfiguration(
                "safer", 42, baseline.configuration.algorithms,
                baseline.configuration.sensor, baseline.configuration.robot,
                baseline.configuration.map,
            ),
            ExperimentSummary(
                "COMPLETED", "done", 12.0, 5.0, 1.0, 20, 20,
                3, 0, 1, 1, 0,
            ),
            (),
        )
        ranking = compare_experiments((baseline, safer))
        self.assertEqual(ranking[0].experiment, "safer")
        self.assertEqual(ranking[0].rank, 1)


if __name__ == "__main__":
    unittest.main()
