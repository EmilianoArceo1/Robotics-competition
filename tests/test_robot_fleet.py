from __future__ import annotations

import unittest

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.simulation_controller import SimulationController
from Logic.Robot import RobotFleet, RobotPhysics


class RobotFleetTests(unittest.TestCase):
    def test_fleet_members_have_independent_physics_and_ids(self) -> None:
        fleet = RobotFleet(RobotPhysics(), 4)
        self.assertEqual(
            [member.robot_id for member in fleet.members],
            ["robot-1", "robot-2", "robot-3", "robot-4"],
        )
        self.assertEqual(len({id(member.physics) for member in fleet.members}), 4)
        self.assertTrue(fleet.members[0].controllable)
        self.assertFalse(fleet.members[1].controllable)

    def test_count_updates_backend_and_map_snapshot(self) -> None:
        map_controller = MapController()
        simulation = SimulationController(
            map_controller, PathPlannerController(), ObjectiveAssignController(),
            BeliefMapController(map_controller.simulation_map),
        )
        simulation.set_robot_count(5)
        self.assertEqual(len(map_controller.snapshot().robots), 5)

        simulation.start()
        self.assertEqual(len(simulation.fleet.members), 5)
        self.assertEqual(len(map_controller.snapshot().robots), 5)
        simulation.stop()
        self.assertEqual(
            simulation.experiment_result.configuration.robot.robot_count,
            5,
        )


if __name__ == "__main__":
    unittest.main()
