from __future__ import annotations

import importlib
import unittest

from Logic.Robot.Physic import RobotPhysics
from Logic.Robot.Sensor import Sensor


AStar = importlib.import_module("Logic.Methods.Path planers.Astar").AStar


class PlanningAndSensorTests(unittest.TestCase):
    def test_astar_uses_same_grid_geometry(self) -> None:
        planner = AStar(
            RobotPhysics(),
            ((0.6, 0.0),),
            grid_size=0.3,
        )
        route = planner.plan_route((0.0, 0.0), (1.2, 0.0))
        self.assertNotIn([0.6, 0.0], route)
        self.assertEqual(route[0], [0.0, 0.0])
        self.assertEqual(route[-1], [1.2, 0.0])

    def test_sensor_occlusion_uses_same_grid_cells(self) -> None:
        sensor = Sensor(detection_radius=3.0, grid_size=0.3)
        detected = sensor.detect(
            (0.0, 0.0, 0.0),
            (
                [[0.6, 0.0], 1],
                [[0.9, 0.0], 0],
            ),
        )
        self.assertIn([[0.6, 0.0], 1], detected)
        self.assertNotIn([[0.9, 0.0], 0], detected)


if __name__ == "__main__":
    unittest.main()
