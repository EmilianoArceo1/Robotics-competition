from __future__ import annotations

import unittest

from Logic.Map.grid_geometry import GridCell, GridGeometry
from Logic.Planning.costmap import PlanningCostmap
from Logic.Planning.path_simplifier import PathSimplifier


def free_costmap(cells: set[GridCell]) -> PlanningCostmap:
    return PlanningCostmap(
        GridGeometry(1.0),
        frozenset(cells),
        frozenset(),
        frozenset(),
        True,
    )


class PathSimplifierTests(unittest.TestCase):
    def test_straight_route_reduces_to_endpoints(self) -> None:
        path = [GridCell(column, 0) for column in range(6)]
        simplified = PathSimplifier(free_costmap(set(path))).simplify(path)
        self.assertEqual(simplified, [path[0], path[-1]])

    def test_diagonal_cannot_cut_blocked_corner(self) -> None:
        start, end = GridCell(0, 0), GridCell(1, 1)
        costmap = free_costmap({start, end, GridCell(0, 1)})
        simplifier = PathSimplifier(costmap)
        self.assertFalse(simplifier.segment_is_safe(start, end))

    def test_simplified_segments_remain_traversable(self) -> None:
        path = [
            GridCell(0, 0), GridCell(1, 0), GridCell(2, 0),
            GridCell(2, 1), GridCell(2, 2),
        ]
        simplifier = PathSimplifier(free_costmap(set(path)))
        simplified = simplifier.simplify(path)
        self.assertTrue(
            all(
                simplifier.segment_is_safe(start, end)
                for start, end in zip(simplified, simplified[1:])
            )
        )


if __name__ == "__main__":
    unittest.main()
