"""Planificador A* para una cuadrícula bidimensional."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import count

from Logic.Robot.Physic import RobotPhysics
from Logic.Robot.Track import Coordinate, CoordinateMatrix, Track
from Logic.Map.grid_geometry import GridCell, GridGeometry
from Logic.Planning.costmap import PlanningCostmap, PlanningCostmapBuilder
from Logic.Planning.path_simplifier import PathSimplifier
from Logic.Planning.route_plan import RoutePlanResult
from Logic.Robot.BeliefMap import BeliefMap

GridPoint = GridCell


class AStar(Track):
    def __init__(
        self,
        robot: RobotPhysics,
        obstacles: tuple[tuple[float, float], ...],
        *,
        map_padding: int = 4,
        grid_size: float = 1.0,
        safety_radius: float = 0.0,
        unknown_is_blocked: bool = True,
        **track_options: float,
    ) -> None:
        super().__init__(robot, **track_options)
        self.grid_size = float(grid_size)
        if not 0.1 <= self.grid_size <= 5.0:
            raise ValueError("grid_size debe estar entre 0.1 y 5.0")
        self.geometry = GridGeometry(self.grid_size)
        physical_radius = (
            (robot.geometry.length / 2.0) ** 2
            + (robot.geometry.width / 2.0) ** 2
        ) ** 0.5
        self.costmap_builder = PlanningCostmapBuilder(
            self.geometry,
            robot_radius=physical_radius,
            safety_radius=safety_radius,
            unknown_is_blocked=unknown_is_blocked,
        )
        self.costmap: PlanningCostmap | None = None
        self.last_plan_result: RoutePlanResult | None = None
        self.blocked = {
            self.geometry.world_to_cell(*coordinate)
            for coordinate in obstacles
        }
        self.map_padding = int(map_padding)
        if self.map_padding < 1:
            raise ValueError("map_padding debe ser mayor o igual que uno")

    def update_belief_map(self, belief_map: BeliefMap) -> PlanningCostmap:
        self.costmap = self.costmap_builder.build(belief_map)
        return self.costmap

    def configure_safety_radius(self, radius: float) -> None:
        self.costmap_builder.configure_safety_radius(radius)
        self.costmap = None

    def _is_traversable(self, cell: GridCell) -> bool:
        if self.costmap is not None:
            return self.costmap.is_traversable(cell)
        return cell not in self.blocked

    def _safe_goal(self, requested: GridCell) -> GridCell:
        if self._is_traversable(requested):
            return requested
        if self.costmap is None:
            raise ValueError("La meta debe estar en espacio libre")
        candidates = (
            cell for cell in self.costmap.free_cells
            if self.costmap.is_traversable(cell)
        )
        try:
            return min(
                candidates,
                key=lambda cell: (
                    abs(cell.column - requested.column)
                    + abs(cell.row - requested.row),
                    (cell.column - requested.column) ** 2
                    + (cell.row - requested.row) ** 2,
                    cell.row,
                    cell.column,
                ),
            )
        except ValueError as error:
            raise ValueError("No existe una meta segura en el costmap") from error

    def _grid_point(self, coordinate: Coordinate) -> GridPoint:
        if len(coordinate) != 2:
            raise ValueError("La coordenada debe ser [x, y]")
        return self.geometry.world_to_cell(
            float(coordinate[0]), float(coordinate[1])
        )

    def _search(
        self, start: Coordinate, goal: Coordinate
    ) -> tuple[list[GridPoint], GridPoint, int]:
        start_point = self._grid_point(start)
        requested_goal = self._grid_point(goal)
        if not self._is_traversable(start_point):
            raise ValueError("El inicio debe estar en espacio libre")
        goal_point = self._safe_goal(requested_goal)
        if start_point == goal_point:
            return [start_point], goal_point, 1

        relevant = (
            (*self.costmap.known_cells, start_point, goal_point)
            if self.costmap is not None
            else (*self.blocked, start_point, goal_point)
        )
        min_x = min(point.column for point in relevant) - self.map_padding
        max_x = max(point.column for point in relevant) + self.map_padding
        min_y = min(point.row for point in relevant) - self.map_padding
        max_y = max(point.row for point in relevant) + self.map_padding

        frontier: list[tuple[int, int, GridPoint]] = []
        order = count()
        heappush(frontier, (0, next(order), start_point))
        came_from: dict[GridPoint, GridPoint | None] = {start_point: None}
        cost: dict[GridPoint, int] = {start_point: 0}
        evaluated_cells = 0

        while frontier:
            _, _, current = heappop(frontier)
            evaluated_cells += 1
            if current == goal_point:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = GridCell(current.column + dx, current.row + dy)
                if (
                    not self._is_traversable(neighbor)
                    or not min_x <= neighbor.column <= max_x
                    or not min_y <= neighbor.row <= max_y
                ):
                    continue
                new_cost = cost[current] + 1
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + abs(neighbor.column - goal_point.column) + abs(
                        neighbor.row - goal_point.row
                    )
                    heappush(frontier, (priority, next(order), neighbor))
                    came_from[neighbor] = current

        if goal_point not in came_from:
            raise ValueError("A* no encontró una ruta hacia la meta")

        path: list[GridPoint] = []
        current: GridPoint | None = goal_point
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path, goal_point, evaluated_cells

    def plan_route_result(
        self, start: Coordinate, goal: Coordinate
    ) -> RoutePlanResult:
        requested_goal = float(goal[0]), float(goal[1])
        try:
            raw_cells, safe_goal_cell, evaluated = self._search(start, goal)
            simplified_cells = (
                PathSimplifier(self.costmap).simplify(raw_cells)
                if self.costmap is not None
                else raw_cells
            )
            raw_path = tuple(
                self.geometry.cell_to_world(cell) for cell in raw_cells
            )
            simplified_path = tuple(
                self.geometry.cell_to_world(cell) for cell in simplified_cells
            )
            result = RoutePlanResult(
                True,
                "route planned",
                requested_goal,
                self.geometry.cell_to_world(safe_goal_cell),
                raw_path,
                simplified_path,
                evaluated,
            )
        except (TypeError, ValueError) as error:
            result = RoutePlanResult(
                False, str(error), requested_goal, None
            )
        self.last_plan_result = result
        return result

    def plan_route(
        self, start: Coordinate, goal: Coordinate
    ) -> CoordinateMatrix:
        result = self.plan_route_result(start, goal)
        if not result.success:
            raise ValueError(result.reason)
        return [[x, y] for x, y in result.simplified_path]


__all__ = ["AStar"]
