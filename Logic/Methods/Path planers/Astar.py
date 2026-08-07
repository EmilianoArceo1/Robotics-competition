"""Planificador A* para una cuadrícula bidimensional."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import count

from Logic.Robot.Physic import RobotPhysics
from Logic.Robot.Track import Coordinate, CoordinateMatrix, Track
from Logic.Map.grid_geometry import GridCell, GridGeometry

GridPoint = GridCell


class AStar(Track):
    def __init__(
        self,
        robot: RobotPhysics,
        obstacles: tuple[tuple[float, float], ...],
        *,
        map_padding: int = 4,
        grid_size: float = 1.0,
        **track_options: float,
    ) -> None:
        super().__init__(robot, **track_options)
        self.grid_size = float(grid_size)
        if not 0.1 <= self.grid_size <= 5.0:
            raise ValueError("grid_size debe estar entre 0.1 y 5.0")
        self.geometry = GridGeometry(self.grid_size)
        self.blocked = {
            self.geometry.world_to_cell(*coordinate)
            for coordinate in obstacles
        }
        self.map_padding = int(map_padding)
        if self.map_padding < 1:
            raise ValueError("map_padding debe ser mayor o igual que uno")

    def _grid_point(self, coordinate: Coordinate) -> GridPoint:
        if len(coordinate) != 2:
            raise ValueError("La coordenada debe ser [x, y]")
        return self.geometry.world_to_cell(
            float(coordinate[0]), float(coordinate[1])
        )

    def plan_route(
        self, start: Coordinate, goal: Coordinate
    ) -> CoordinateMatrix:
        start_point = self._grid_point(start)
        goal_point = self._grid_point(goal)
        if start_point in self.blocked or goal_point in self.blocked:
            raise ValueError("El inicio y la meta deben estar en espacio libre")
        if start_point == goal_point:
            return [[
                *self.geometry.cell_to_world(start_point),
            ]]

        relevant = (*self.blocked, start_point, goal_point)
        min_x = min(point.column for point in relevant) - self.map_padding
        max_x = max(point.column for point in relevant) + self.map_padding
        min_y = min(point.row for point in relevant) - self.map_padding
        max_y = max(point.row for point in relevant) + self.map_padding

        frontier: list[tuple[int, int, GridPoint]] = []
        order = count()
        heappush(frontier, (0, next(order), start_point))
        came_from: dict[GridPoint, GridPoint | None] = {start_point: None}
        cost: dict[GridPoint, int] = {start_point: 0}

        while frontier:
            _, _, current = heappop(frontier)
            if current == goal_point:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = GridCell(current.column + dx, current.row + dy)
                if (
                    neighbor in self.blocked
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
        return [
            [*self.geometry.cell_to_world(cell)]
            for cell in path
        ]


__all__ = ["AStar"]
