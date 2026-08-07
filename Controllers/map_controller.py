"""Controlador que conecta la lógica del mapa con su vista."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi

from Logic.Map.maps import Coordinate, SimulationMap


@dataclass(frozen=True, slots=True)
class MapSnapshot:
    obstacles: tuple[Coordinate, ...]
    robot_position: Coordinate
    robot_heading: float
    sensor_radius: float | None
    sensor_fov: float | None
    sensor_visibility: tuple[Coordinate, ...]
    frontiers: tuple[Coordinate, ...]
    route: tuple[Coordinate, ...]
    active_waypoint_index: int


class MapController:
    def __init__(self, simulation_map: SimulationMap | None = None) -> None:
        self._map = simulation_map or SimulationMap()
        self._robot_heading = 0.0
        self._sensor_radius: float | None = None
        self._sensor_fov: float | None = None
        self._sensor_visibility: tuple[Coordinate, ...] = ()
        self._frontiers: tuple[Coordinate, ...] = ()
        self._show_frontiers = False
        self._route: tuple[Coordinate, ...] = ()
        self._active_waypoint_index = 0
        self._show_route = False
        self._show_obstacles = True

    @property
    def simulation_map(self) -> SimulationMap:
        return self._map

    def snapshot(self) -> MapSnapshot:
        return MapSnapshot(
            obstacles=self._map.obstacles,
            robot_position=self._map.robot_world_position,
            robot_heading=self._robot_heading,
            sensor_radius=self._sensor_radius,
            sensor_fov=self._sensor_fov,
            sensor_visibility=self._sensor_visibility,
            frontiers=self._frontiers if self._show_frontiers else (),
            route=self._route if self._show_route else (),
            active_waypoint_index=self._active_waypoint_index,
        )

    def configure_sensor(self, radius: float, field_of_view: float) -> None:
        radius, field_of_view = float(radius), float(field_of_view)
        if not isfinite(radius) or radius <= 0.0:
            raise ValueError("El radio del sensor debe ser positivo")
        if not isfinite(field_of_view) or not 0.0 < field_of_view <= 360.0:
            raise ValueError("El FOV debe estar entre 0 y 360 grados")
        self._sensor_radius = radius
        self._sensor_fov = field_of_view

    def update_frontiers(self, coordinates: tuple[Coordinate, ...]) -> None:
        self._frontiers = tuple(
            self._map.local_to_world(coordinate) for coordinate in coordinates
        )

    def update_sensor_visibility(
        self, coordinates: tuple[Coordinate, ...]
    ) -> None:
        self._sensor_visibility = tuple(
            self._map.local_to_world(coordinate) for coordinate in coordinates
        )

    def update_route(
        self,
        coordinates: tuple[Coordinate, ...],
        active_waypoint_index: int,
    ) -> None:
        if active_waypoint_index < 0:
            raise ValueError("active_waypoint_index no puede ser negativo")
        self._route = tuple(
            self._map.local_to_world(coordinate) for coordinate in coordinates
        )
        self._active_waypoint_index = int(active_waypoint_index)

    def set_show_route(self, visible: bool) -> None:
        self._show_route = bool(visible)

    @property
    def show_route(self) -> bool:
        return self._show_route

    def set_show_frontiers(self, visible: bool) -> None:
        self._show_frontiers = bool(visible)

    @property
    def show_frontiers(self) -> bool:
        return self._show_frontiers

    def set_show_obstacles(self, visible: bool) -> None:
        self._show_obstacles = bool(visible)

    @property
    def show_obstacles(self) -> bool:
        return self._show_obstacles

    def update_robot_pose(self, x: float, y: float, theta: float) -> None:
        """Actualiza la pose relativa; ``theta`` se expresa en radianes."""
        values = (float(x), float(y), float(theta))
        if not all(isfinite(value) for value in values):
            raise ValueError("La pose del robot debe contener números finitos")
        self._map.update_robot_position(values[:2])
        self._robot_heading = (values[2] + pi) % (2.0 * pi) - pi

    def reset_robot(self) -> None:
        self._map.reset_robot()
        self._robot_heading = 0.0
        self._frontiers = ()
        self._sensor_visibility = ()
        self._route = ()
        self._active_waypoint_index = 0

    def scene_bounds(
        self, padding: float = 1.0
    ) -> tuple[float, float, float, float]:
        if padding < 0.0:
            raise ValueError("padding no puede ser negativo")
        snapshot = self.snapshot()
        coordinates = (
            *snapshot.obstacles,
            *snapshot.frontiers,
            *snapshot.sensor_visibility,
            *snapshot.route,
            snapshot.robot_position,
        )
        xs = [coordinate[0] for coordinate in coordinates]
        ys = [coordinate[1] for coordinate in coordinates]
        return (
            min(xs) - padding,
            min(ys) - padding,
            max(xs) + padding,
            max(ys) + padding,
        )
