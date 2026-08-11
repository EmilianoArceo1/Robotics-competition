from __future__ import annotations

import unittest
from pathlib import Path

from Infrastructure.Maps import NpyOccupancyMapLoader
from Logic.Robot.CollisionChecker import CollisionChecker
from Logic.Robot.Sensor import Sensor


ASSET = (
    Path(__file__).resolve().parents[1]
    / "Assets" / "maps" / "env3" / "occ_map.npy"
)


class CompetitionMapTests(unittest.TestCase):
    def test_env3_is_imported_with_metric_cells_and_safe_origin(self) -> None:
        simulation_map = NpyOccupancyMapLoader().load(str(ASSET))
        self.assertEqual(simulation_map.obstacle_size, 0.5)
        self.assertGreater(len(simulation_map.obstacles), 2_000)
        self.assertLess(len(simulation_map.obstacles), 4_000)
        checker = CollisionChecker(
            simulation_map.local_obstacles,
            robot_radius=0.31,
            safety_radius=0.2,
            obstacle_size=simulation_map.obstacle_size,
        )
        self.assertFalse(checker.check_position((0.0, 0.0)).collision)

    def test_dense_map_rasterization_preserves_walls(self) -> None:
        simulation_map = NpyOccupancyMapLoader().load(str(ASSET))
        matrix = simulation_map.occupancy_matrix(padding=2.0, cell_size=1.0)
        self.assertTrue(any(cell[1] == 1 for cell in matrix))
        self.assertTrue(any(cell[1] == 0 for cell in matrix))

    def test_dense_occlusion_uses_physical_resolution(self) -> None:
        wall = tuple((2.0, index * 0.5 - 150.0) for index in range(600))
        sensor = Sensor(detection_radius=5.0, grid_size=1.6, field_of_view=2.0)
        scan = sensor.scan(
            (0.0, 0.0, 0.0), (), angular_resolution=1.0,
            occluders=wall, obstacle_size=0.5,
        )
        self.assertAlmostEqual(scan.visibility_polygon[2][0], 1.75, delta=0.06)


if __name__ == "__main__":
    unittest.main()
