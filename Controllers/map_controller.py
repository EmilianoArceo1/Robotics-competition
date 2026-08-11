"""Controlador que conecta la lógica del mapa con su vista."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi

from Logic.Map.maps import Coordinate, SimulationMap


@dataclass(frozen=True, slots=True)
class MapSnapshot:
    obstacles: tuple[Coordinate, ...]
    obstacle_size: float
    robot_position: Coordinate
    robot_heading: float
    sensor_radius: float | None
    sensor_fov: float | None
    sensor_visibility: tuple[Coordinate, ...]
    frontiers: tuple[Coordinate, ...]
    frontier_clusters: tuple[tuple[Coordinate, ...], ...]
    route: tuple[Coordinate, ...]
    active_waypoint_index: int
    robots: tuple["RobotPoseSnapshot", ...]


@dataclass(frozen=True, slots=True)
class RobotPoseSnapshot:
    robot_id: str
    position: Coordinate
    heading: float
    controllable: bool


class MapController:
    def __init__(self, simulation_map: SimulationMap | None = None) -> None:
        self._map = simulation_map or SimulationMap()
        self._robot_heading = 0.0
        self._robot_poses: dict[str, tuple[float, float, float, bool]] = {
            "robot-1": (0.0, 0.0, 0.0, True)
        }
        self._sensor_radius: float | None = None
        self._sensor_fov: float | None = None
        self._sensor_visibility: tuple[Coordinate, ...] = ()
        self._frontiers: tuple[Coordinate, ...] = ()
        self._show_frontiers = False
        self._frontier_clusters: tuple[tuple[Coordinate, ...], ...] = ()
        self._show_clusters = False
        self._route: tuple[Coordinate, ...] = ()
        self._active_waypoint_index = 0
        self._show_route = False
        self._show_obstacles = True

    @property
    def simulation_map(self) -> SimulationMap:
        return self._map

    def replace_map(self, simulation_map: SimulationMap) -> None:
        self._map = simulation_map
        self._robot_poses = {"robot-1": (0.0, 0.0, 0.0, True)}
        self.reset_robot()

    def configure_robot_poses(self, poses) -> None:
        start_x, start_y = self._map.robot_start_world
        self._robot_poses = {
            f"robot-{index + 1}": (float(x) - start_x, float(y) - start_y, float(theta), index == 0)
            for index, (x, y, theta) in enumerate(poses)
        }
        primary = self._robot_poses["robot-1"]
        self.update_robot_pose(*primary[:3], "robot-1")

    def snapshot(self) -> MapSnapshot:
        return MapSnapshot(
            obstacles=self._map.obstacles,
            obstacle_size=self._map.obstacle_size,
            robot_position=self._map.robot_world_position,
            robot_heading=self._robot_heading,
            sensor_radius=self._sensor_radius,
            sensor_fov=self._sensor_fov,
            sensor_visibility=self._sensor_visibility,
            frontiers=self._frontiers if self._show_frontiers else (),
            frontier_clusters=(
                self._frontier_clusters if self._show_clusters else ()
            ),
            route=self._route if self._show_route else (),
            active_waypoint_index=self._active_waypoint_index,
            robots=tuple(
                RobotPoseSnapshot(
                    robot_id,
                    self._map.local_to_world((pose[0], pose[1])),
                    pose[2],
                    pose[3],
                )
                for robot_id, pose in self._robot_poses.items()
            ),
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

    def update_frontier_clusters(
        self, clusters: tuple[tuple[Coordinate, ...], ...]
    ) -> None:
        self._frontier_clusters = tuple(
            tuple(self._map.local_to_world(point) for point in cluster)
            for cluster in clusters
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

    def set_show_clusters(self, visible: bool) -> None:
        self._show_clusters = bool(visible)

    @property
    def show_clusters(self) -> bool:
        return self._show_clusters

    def set_show_obstacles(self, visible: bool) -> None:
        self._show_obstacles = bool(visible)

    @property
    def show_obstacles(self) -> bool:
        return self._show_obstacles

    def update_robot_pose(
        self, x: float, y: float, theta: float, robot_id: str = "robot-1"
    ) -> None:
        """Actualiza la pose relativa; ``theta`` se expresa en radianes."""
        values = (float(x), float(y), float(theta))
        if not all(isfinite(value) for value in values):
            raise ValueError("La pose del robot debe contener números finitos")
        heading = (values[2] + pi) % (2.0 * pi) - pi
        controllable = self._robot_poses.get(robot_id, (0, 0, 0, False))[3]
        self._robot_poses[str(robot_id)] = (*values[:2], heading, controllable)
        if robot_id == "robot-1":
            self._map.update_robot_position(values[:2])
            self._robot_heading = heading

    def update_fleet(self, members) -> None:
        self._robot_poses = {
            member.robot_id: (
                member.physics.state.x,
                member.physics.state.y,
                member.physics.state.theta,
                member.controllable,
            )
            for member in members
        }
        primary = self._robot_poses["robot-1"]
        self.update_robot_pose(*primary[:3], "robot-1")

    def configure_robot_preview(self, count: int, spacing: float = 0.8) -> None:
        if not 1 <= int(count) <= 20:
            raise ValueError("El número de robots debe estar entre 1 y 20")
        self._robot_poses = {
            f"robot-{index + 1}": (
                0.0, -index * float(spacing), 0.0, index == 0
            )
            for index in range(int(count))
        }

    def reset_robot(self) -> None:
        self._map.reset_robot()
        self._robot_heading = 0.0
        count = len(self._robot_poses)
        self.configure_robot_preview(count)
        self._frontiers = ()
        self._frontier_clusters = ()
        self._sensor_visibility = ()
        self._route = ()
        self._active_waypoint_index = 0

    def scene_bounds(
        self, padding: float = 1.0
    ) -> tuple[float, float, float, float]:
        if padding < 0.0:
            raise ValueError("padding no puede ser negativo")
        snapshot = self.snapshot()
        if self._map.world_bounds is not None:
            x_min, y_min, x_max, y_max = self._map.world_bounds
            return x_min - padding, y_min - padding, x_max + padding, y_max + padding
        coordinates = (
            *snapshot.obstacles,
            *snapshot.frontiers,
            *(point for cluster in snapshot.frontier_clusters for point in cluster),
            *snapshot.sensor_visibility,
            *snapshot.route,
            *(robot.position for robot in snapshot.robots),
        )
        xs = [coordinate[0] for coordinate in coordinates]
        ys = [coordinate[1] for coordinate in coordinates]
        return (
            min(xs) - padding,
            min(ys) - padding,
            max(xs) + padding,
            max(ys) + padding,
        )
