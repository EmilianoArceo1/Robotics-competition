from __future__ import annotations

import unittest

from Logic.Map.grid_geometry import GridCell, GridGeometry
from Logic.Map.maps import SimulationMap


class GridGeometryTests(unittest.TestCase):
    def test_round_trip_for_supported_decimal_resolutions(self) -> None:
        for resolution in (0.1, 0.3, 0.5, 0.7, 1.0, 1.6, 2.0):
            with self.subTest(resolution=resolution):
                geometry = GridGeometry(resolution)
                for column, row in ((0, 0), (3, -4), (-7, 5)):
                    cell = GridCell(column, row)
                    self.assertEqual(
                        geometry.world_to_cell(*geometry.cell_to_world(cell)),
                        cell,
                    )

    def test_adjacent_cells_share_exact_boundary(self) -> None:
        geometry = GridGeometry(0.3)
        left = geometry.cell_bounds(GridCell(2, 0))
        right = geometry.cell_bounds(GridCell(3, 0))
        self.assertEqual(left[1], right[0])

    def test_occupancy_matrix_uses_grid_centers(self) -> None:
        matrix = SimulationMap(obstacles=((0.6, -0.3),)).occupancy_matrix(
            padding=1.0, cell_size=0.3
        )
        occupied = [cell[0] for cell in matrix if cell[1] == 1]
        self.assertEqual(occupied, [[0.6, -0.3]])


if __name__ == "__main__":
    unittest.main()
