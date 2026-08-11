from __future__ import annotations

import importlib
import unittest

from Logic.Robot.Physic import RobotPhysics
from Logic.Robot.Sensor import Sensor
from Logic.Map.maps import SimulationMap


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

    def test_physical_occlusion_does_not_change_with_grid_size(self) -> None:
        endpoints = []
        for grid_size in (0.2, 0.5, 1.0, 1.6):
            sensor = Sensor(
                detection_radius=5.0,
                field_of_view=2.0,
                grid_size=grid_size,
            )
            scan = sensor.scan(
                (0.0, 0.0, 0.0),
                (),
                angular_resolution=1.0,
                occluders=((2.0, 0.0),),
                obstacle_size=1.0,
            )
            endpoints.append(scan.visibility_polygon[2][0])

        for endpoint in endpoints:
            self.assertAlmostEqual(endpoint, 1.5, places=6)

    def test_obstacle_rasterization_preserves_physical_footprint(self) -> None:
        simulation_map = SimulationMap(obstacles=((2.0, 0.0),))
        for grid_size in (0.2, 0.5, 1.0, 1.6):
            occupied = [
                cell for cell in simulation_map.occupancy_matrix(
                    padding=1.0, cell_size=grid_size
                )
                if cell[1] == 1
            ]
            self.assertTrue(occupied)
            for cell in occupied:
                x, y = cell[0]
                half = grid_size / 2.0
                self.assertGreater(x + half, 1.5)
                self.assertLess(x - half, 2.5)
                self.assertGreater(y + half, -0.5)
                self.assertLess(y - half, 0.5)


if __name__ == "__main__":
    unittest.main()
