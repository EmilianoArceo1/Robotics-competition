from __future__ import annotations

import unittest

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.simulation_controller import SimulationController
from Logic.Navigation import NavigationState


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
        self.assertIn(
            simulation.navigation_snapshot.state,
            {
                NavigationState.FOLLOWING,
                NavigationState.GOAL_REACHED,
                NavigationState.EXHAUSTED,
            },
        )

    def test_planner_failure_does_not_stop_simulation(self) -> None:
        map_controller = MapController()
        simulation = SimulationController(
            map_controller,
            PathPlannerController(),
            ObjectiveAssignController(),
            BeliefMapController(map_controller.simulation_map),
        )
        simulation.start()

        def fail(_start, _goal):
            raise RuntimeError("synthetic planner failure")

        simulation.track.plan_route = fail
        simulation.step(0.05)

        self.assertTrue(simulation.running)
        self.assertEqual(
            simulation.navigation_snapshot.state,
            NavigationState.EXHAUSTED,
        )
        self.assertIn(
            "synthetic planner failure",
            simulation.navigation_snapshot.reason,
        )


if __name__ == "__main__":
    unittest.main()
