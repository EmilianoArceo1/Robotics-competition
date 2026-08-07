from __future__ import annotations

import unittest

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.simulation_controller import SimulationController


class SimulationPipelineTests(unittest.TestCase):
    def test_tick_runs_perception_navigation_and_motion(self) -> None:
        map_controller = MapController()
        belief_controller = BeliefMapController(
            map_controller.simulation_map
        )
        simulation = SimulationController(
            map_controller,
            PathPlannerController(),
            ObjectiveAssignController(),
            belief_controller,
        )
        simulation.set_grid_size(0.3)
        simulation.start()
        simulation.step(0.05)

        self.assertTrue(simulation.running)
        self.assertGreater(len(simulation.control.belief_map), 0)
        self.assertGreater(len(belief_controller.cells), 0)
        self.assertIsNotNone(simulation.control.last_scan)
        self.assertIsNotNone(simulation.track)
        self.assertEqual(
            map_controller.snapshot().robot_position,
            (
                simulation.robot.state.x,
                simulation.robot.state.y,
            ),
        )


if __name__ == "__main__":
    unittest.main()
