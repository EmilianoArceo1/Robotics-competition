"""Mapa de referencia del simulador y transformación de coordenadas."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from math import isfinite

from .grid_geometry import GridCell, GridGeometry

Coordinate = tuple[float, float]

# Coordenadas globales, fijas y destinadas a representar los obstáculos.
# Este arreglo puede sustituirse cuando se defina el diseño final del mapa.
DEFAULT_OBSTACLES: tuple[Coordinate, ...] = (
    (2.0, 0.0),
    (2.0, 1.0),
    (2.0, 2.0),
    (-1.0, 3.0),
    (0.0, 3.0),
    (1.0, 3.0),
)


def _coordinate(value: Sequence[float], name: str) -> Coordinate:
    if len(value) != 2:
        raise ValueError(f"{name} debe contener exactamente [x, y]")
    x, y = float(value[0]), float(value[1])
    if not isfinite(x) or not isfinite(y):
        raise ValueError(f"{name} debe contener números finitos")
    return x, y


class SimulationMap:
    """Separa el mapa global de la posición relativa del robot.

    Los obstáculos nunca se desplazan. La posición inicial global del robot
    define el origen de su marco local, por lo que su posición relativa siempre
    comienza en ``(0, 0)`` independientemente de dónde aparezca en el mapa.
    """

    def __init__(
        self,
        obstacles: Iterable[Sequence[float]] = DEFAULT_OBSTACLES,
        *,
        robot_start_world: Sequence[float] = (0.0, 0.0),
        obstacle_size: float = 1.0,
        world_bounds: Sequence[float] | None = None,
    ) -> None:
        self._obstacles: tuple[Coordinate, ...] = tuple(
            _coordinate(point, "Cada obstáculo") for point in obstacles
        )
        if len(set(self._obstacles)) != len(self._obstacles):
            raise ValueError("Las coordenadas de obstáculos no deben repetirse")
        self._robot_start_world = _coordinate(
            robot_start_world, "robot_start_world"
        )
        self._obstacle_size = float(obstacle_size)
        if not isfinite(self._obstacle_size) or self._obstacle_size <= 0.0:
            raise ValueError("obstacle_size debe ser positivo")
        self._robot_position: Coordinate = (0.0, 0.0)
        self._world_bounds = None if world_bounds is None else tuple(float(v) for v in world_bounds)
        if self._world_bounds is not None and (len(self._world_bounds) != 4 or not all(isfinite(v) for v in self._world_bounds)):
            raise ValueError("world_bounds debe contener cuatro números finitos")
        if self._world_bounds is not None and (self._world_bounds[0] >= self._world_bounds[2] or self._world_bounds[1] >= self._world_bounds[3]):
            raise ValueError("world_bounds debe seguir [x_min, y_min, x_max, y_max]")

    @property
    def obstacles(self) -> tuple[Coordinate, ...]:
        """Coordenadas globales e inmutables de los obstáculos."""
        return self._obstacles

    @property
    def obstacle_size(self) -> float:
        return self._obstacle_size

    @property
    def world_bounds(self) -> tuple[float, float, float, float] | None:
        return self._world_bounds

    @property
    def local_obstacles(self) -> tuple[Coordinate, ...]:
        return tuple(self.world_to_local(point) for point in self._obstacles)

    @property
    def robot_start_world(self) -> Coordinate:
        return self._robot_start_world

    @property
    def robot_position(self) -> Coordinate:
        """Posición relativa al punto donde comenzó el robot."""
        return self._robot_position

    @property
    def robot_world_position(self) -> Coordinate:
        """Posición global para dibujar el robot junto a los obstáculos."""
        return (
            self._robot_start_world[0] + self._robot_position[0],
            self._robot_start_world[1] + self._robot_position[1],
        )

    def update_robot_position(self, position: Sequence[float]) -> None:
        """Actualiza la posición relativa del robot."""
        self._robot_position = _coordinate(position, "position")

    def local_to_world(self, coordinate: Sequence[float]) -> Coordinate:
        """Convierte una coordenada relativa al marco global del mapa."""
        local_x, local_y = _coordinate(coordinate, "coordinate")
        return (
            self._robot_start_world[0] + local_x,
            self._robot_start_world[1] + local_y,
        )

    def world_to_local(self, coordinate: Sequence[float]) -> Coordinate:
        world_x, world_y = _coordinate(coordinate, "coordinate")
        return world_x - self._robot_start_world[0], world_y - self._robot_start_world[1]

    def reset_robot(self) -> None:
        """Devuelve al robot al origen relativo de su recorrido."""
        self._robot_position = (0.0, 0.0)

    @property
    def sensor_matrix(self) -> list[list[object]]:
        """Obstáculos ocupados expresados en el marco local del robot.

        La salida respeta el contrato de ``Sensor.detect``:
        ``[[[x, y], 1], ...]``.
        """
        start_x, start_y = self._robot_start_world
        return [
            [[obstacle_x - start_x, obstacle_y - start_y], 1]
            for obstacle_x, obstacle_y in self._obstacles
        ]

    def occupancy_matrix(
        self, padding: float = 4.0, cell_size: float = 1.0
    ) -> list[list[object]]:
        """Devuelve una cuadrícula local completa para simular el sensor."""
        cell_size = float(cell_size)
        if padding <= 0.0:
            raise ValueError("padding debe ser positivo")
        if not isfinite(cell_size) or not 0.1 <= cell_size <= 5.0:
            raise ValueError("cell_size debe estar entre 0.1 y 5.0")
        geometry = GridGeometry(cell_size)
        local_obstacle_centers = self.local_obstacles
        half = self._obstacle_size / 2.0
        epsilon = min(cell_size, self._obstacle_size) * 1e-9
        local_obstacles: set[GridCell] = set()
        for obstacle_x, obstacle_y in local_obstacle_centers:
            lower = geometry.world_to_cell(
                obstacle_x - half + epsilon,
                obstacle_y - half + epsilon,
            )
            upper = geometry.world_to_cell(
                obstacle_x + half - epsilon,
                obstacle_y + half - epsilon,
            )
            local_obstacles.update(
                GridCell(column, row)
                for row in range(lower.row, upper.row + 1)
                for column in range(lower.column, upper.column + 1)
            )
        robot_x, robot_y = self._robot_position
        relevant = (
            *local_obstacles,
            geometry.world_to_cell(robot_x, robot_y),
            GridCell(0, 0),
        )
        margin = max(1, round(float(padding) / cell_size))
        min_x = min(point.column for point in relevant) - margin
        max_x = max(point.column for point in relevant) + margin
        min_y = min(point.row for point in relevant) - margin
        max_y = max(point.row for point in relevant) + margin
        matrix: list[list[object]] = []
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                cell = GridCell(x, y)
                matrix.append(
                    [[*geometry.cell_to_world(cell)], 1 if cell in local_obstacles else 0]
                )
        return matrix
