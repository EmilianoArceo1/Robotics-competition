"""Costmap inmutable derivado de observaciones del Belief Map."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot

from Logic.Map.grid_geometry import GridCell, GridGeometry
from Logic.Robot.BeliefMap import BeliefMap


@dataclass(frozen=True, slots=True)
class PlanningCostmap:
    geometry: GridGeometry
    free_cells: frozenset[GridCell]
    occupied_cells: frozenset[GridCell]
    inflated_cells: frozenset[GridCell]
    unknown_is_blocked: bool = True

    @property
    def known_cells(self) -> frozenset[GridCell]:
        return self.free_cells | self.occupied_cells

    def is_traversable(self, cell: GridCell) -> bool:
        if cell in self.inflated_cells or cell in self.occupied_cells:
            return False
        return cell in self.free_cells if self.unknown_is_blocked else True


class PlanningCostmapBuilder:
    def __init__(
        self,
        geometry: GridGeometry,
        *,
        robot_radius: float,
        safety_radius: float = 0.0,
        obstacle_size: float = 1.0,
        unknown_is_blocked: bool = True,
    ) -> None:
        self.geometry = geometry
        self.robot_radius = float(robot_radius)
        self.safety_radius = float(safety_radius)
        self.obstacle_size = float(obstacle_size)
        self.unknown_is_blocked = bool(unknown_is_blocked)
        if min(self.robot_radius, self.safety_radius, self.obstacle_size) < 0.0:
            raise ValueError("Los radios y tamaños no pueden ser negativos")
        if self.obstacle_size <= 0.0:
            raise ValueError("obstacle_size debe ser positivo")

    @property
    def clearance(self) -> float:
        return self.robot_radius + self.safety_radius

    def configure_safety_radius(self, radius: float) -> None:
        value = float(radius)
        if value < 0.0:
            raise ValueError("safety_radius no puede ser negativo")
        self.safety_radius = value

    def build(self, belief_map: BeliefMap) -> PlanningCostmap:
        free: set[GridCell] = set()
        occupied: set[GridCell] = set()
        for coordinate, value in belief_map.matrix:
            cell = self.geometry.world_to_cell(
                float(coordinate[0]), float(coordinate[1])
            )
            if int(value) == 1:
                occupied.add(cell)
            elif int(value) == 0:
                free.add(cell)

        inflated: set[GridCell] = set(occupied)
        reach = ceil(
            (self.obstacle_size / 2.0 + self.clearance)
            / self.geometry.resolution
        )
        resolution = self.geometry.resolution
        obstacle_half = self.obstacle_size / 2.0
        for obstacle in occupied:
            for row_offset in range(-reach, reach + 1):
                for column_offset in range(-reach, reach + 1):
                    dx = max(
                        abs(column_offset) * resolution - obstacle_half,
                        0.0,
                    )
                    dy = max(
                        abs(row_offset) * resolution - obstacle_half,
                        0.0,
                    )
                    if hypot(dx, dy) <= self.clearance + 1e-12:
                        inflated.add(
                            GridCell(
                                obstacle.column + column_offset,
                                obstacle.row + row_offset,
                            )
                        )
        return PlanningCostmap(
            self.geometry,
            frozenset(free),
            frozenset(occupied),
            frozenset(inflated),
            self.unknown_is_blocked,
        )
