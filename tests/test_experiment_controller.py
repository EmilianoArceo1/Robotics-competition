from __future__ import annotations

import unittest

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.simulation_controller import SimulationController


class ExperimentControllerTests(unittest.TestCase):
    @staticmethod
    def build_simulation() -> SimulationController:
        map_controller = MapController()
        return SimulationController(
            map_controller, PathPlannerController(), ObjectiveAssignController(),
            BeliefMapController(map_controller.simulation_map),
        )

    def test_simulation_captures_configuration_and_trajectory(self) -> None:
        simulation = self.build_simulation()
        simulation.configure_experiment(name="repeatable", seed=73)
        simulation.set_grid_size(0.5)
        simulation.start()
        simulation.step(0.05)
        simulation.stop()

        result = simulation.experiment_result
        self.assertIsNotNone(result)
        self.assertEqual(result.configuration.name, "repeatable")
        self.assertEqual(result.configuration.seed, 73)
        self.assertEqual(result.configuration.sensor.grid_size, 0.5)
        self.assertEqual(len(result.trajectory), 1)
        self.assertEqual(result.summary.outcome, "ABORTED")

    def test_same_configuration_produces_same_initial_trace(self) -> None:
        traces = []
        for _ in range(2):
            simulation = self.build_simulation()
            simulation.configure_experiment(name="same", seed=11)
            simulation.start()
            for _ in range(3):
                simulation.step(0.05)
            simulation.stop()
            traces.append(simulation.experiment_result.trajectory)
        self.assertEqual(traces[0], traces[1])

    def test_reset_preserves_the_finished_experiment(self) -> None:
        simulation = self.build_simulation()
        simulation.start()
        simulation.step(0.05)
        simulation.reset()

        result = simulation.experiment_result
        self.assertFalse(simulation.running)
        self.assertIsNotNone(result)
        self.assertEqual(len(result.trajectory), 1)
        self.assertEqual(result.summary.outcome, "ABORTED")


if __name__ == "__main__":
    unittest.main()
