from __future__ import annotations

import importlib
import unittest

from Logic.Map.grid_geometry import GridCell, GridGeometry
from Logic.Planning.costmap import PlanningCostmapBuilder
from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotPhysics


AStar = importlib.import_module("Logic.Methods.Path planers.Astar").AStar


class PlanningCostmapTests(unittest.TestCase):
    def test_obstacle_is_inflated_by_robot_and_safety_radius(self) -> None:
        geometry = GridGeometry(0.5)
        belief = BeliefMap(
            [
                [[column * 0.5, row * 0.5], 0]
                for row in range(-4, 5)
                for column in range(-4, 5)
            ]
        )
        belief.update(([[0.0, 0.0], 1],))
        costmap = PlanningCostmapBuilder(
            geometry, robot_radius=0.3, safety_radius=0.2
        ).build(belief)
        self.assertIn(GridCell(0, 0), costmap.inflated_cells)
        self.assertIn(GridCell(2, 0), costmap.inflated_cells)
        self.assertNotIn(GridCell(3, 0), costmap.inflated_cells)

    def test_unknown_cells_are_blocked_by_default(self) -> None:
        costmap = PlanningCostmapBuilder(
            GridGeometry(1.0), robot_radius=0.2
        ).build(BeliefMap(([[0.0, 0.0], 0],)))
        self.assertTrue(costmap.is_traversable(GridCell(0, 0)))
        self.assertFalse(costmap.is_traversable(GridCell(1, 0)))

    def test_safety_radius_can_be_reconfigured(self) -> None:
        builder = PlanningCostmapBuilder(
            GridGeometry(0.5), robot_radius=0.2, safety_radius=0.1
        )
        self.assertAlmostEqual(builder.clearance, 0.3)
        builder.configure_safety_radius(0.8)
        self.assertAlmostEqual(builder.clearance, 1.0)

    def test_astar_projects_inflated_goal_to_safe_free_cell(self) -> None:
        robot = RobotPhysics()
        planner = AStar(robot, (), grid_size=0.5, safety_radius=0.2)
        observations = [
            [[column * 0.5, row * 0.5], 0]
            for row in range(-4, 5)
            for column in range(-4, 5)
        ]
        observations.append([[1.5, 0.0], 1])
        costmap = planner.update_belief_map(BeliefMap(observations))
        route = planner.plan_route((0.0, 0.0), (1.5, 0.0))
        endpoint = planner.geometry.world_to_cell(*route[-1])
        self.assertTrue(costmap.is_traversable(endpoint))
        self.assertNotEqual(route[-1], [1.5, 0.0])
        self.assertTrue(planner.last_plan_result.success)
        self.assertGreaterEqual(
            len(planner.last_plan_result.raw_path),
            len(planner.last_plan_result.simplified_path),
        )
        self.assertGreater(planner.last_plan_result.evaluated_cells, 0)


if __name__ == "__main__":
    unittest.main()
