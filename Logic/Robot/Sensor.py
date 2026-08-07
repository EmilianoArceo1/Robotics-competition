"""Modelo configurable de un sensor bidimensional."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from math import atan2, ceil, cos, degrees, hypot, isfinite, radians, sin

from .Physic import RobotState
from Logic.Map.grid_geometry import GridCell, GridGeometry

SensorCell = list[object]
SensorMatrix = list[SensorCell]
RobotPose = RobotState | Sequence[float]


@dataclass(frozen=True, slots=True)
class SensorScan:
    detected: SensorMatrix
    visibility_polygon: tuple[tuple[float, float], ...]


def _angular_difference(angle_a: float, angle_b: float) -> float:
    return (angle_a - angle_b + 180.0) % 360.0 - 180.0


@dataclass(slots=True)
class Sensor:
    sensor_type: str = "lidar"
    detection_radius: float = 10.0
    field_of_view: float = 360.0
    grid_size: float = 1.0

    def __post_init__(self) -> None:
        self.sensor_type = str(self.sensor_type).strip().lower()
        self.detection_radius = float(self.detection_radius)
        self.field_of_view = float(self.field_of_view)
        self.grid_size = float(self.grid_size)
        if not self.sensor_type:
            raise ValueError("sensor_type no puede estar vacío")
        if not isfinite(self.detection_radius) or self.detection_radius <= 0.0:
            raise ValueError("detection_radius debe ser un número positivo")
        if not isfinite(self.field_of_view) or not 0.0 < self.field_of_view <= 360.0:
            raise ValueError("field_of_view debe estar entre 0 y 360 grados")
        if not isfinite(self.grid_size) or not 0.1 <= self.grid_size <= 5.0:
            raise ValueError("grid_size debe estar entre 0.1 y 5.0")

    def configure_field_of_view(self, field_of_view: float) -> None:
        value = float(field_of_view)
        if not isfinite(value) or not 0.0 < value <= 360.0:
            raise ValueError("field_of_view debe estar entre 0 y 360 grados")
        self.field_of_view = value

    def configure_grid_size(self, grid_size: float) -> None:
        value = float(grid_size)
        if not isfinite(value) or not 0.1 <= value <= 5.0:
            raise ValueError("grid_size debe estar entre 0.1 y 5.0")
        self.grid_size = value

    @staticmethod
    def _pose_values(robot_pose: RobotPose) -> tuple[float, float, float]:
        if isinstance(robot_pose, RobotState):
            return robot_pose.x, robot_pose.y, degrees(robot_pose.theta)
        if len(robot_pose) not in (2, 3):
            raise ValueError("La pose debe ser [x, y] o [x, y, theta]")
        x, y = float(robot_pose[0]), float(robot_pose[1])
        theta = float(robot_pose[2]) if len(robot_pose) == 3 else 0.0
        if not all(isfinite(value) for value in (x, y, theta)):
            raise ValueError("La pose debe contener números finitos")
        return x, y, degrees(theta)

    @staticmethod
    def _cell_values(cell: Sequence[object]) -> tuple[float, float, int]:
        if len(cell) != 2:
            raise ValueError("Cada celda debe tener la forma [[x, y], valor]")
        coordinate, raw_value = cell
        if not isinstance(coordinate, Sequence) or isinstance(
            coordinate, (str, bytes)
        ):
            raise ValueError("La coordenada de una celda debe ser [x, y]")
        if len(coordinate) != 2:
            raise ValueError("Cada coordenada del entorno debe ser [x, y]")
        x, y = float(coordinate[0]), float(coordinate[1])
        if not isfinite(x) or not isfinite(y):
            raise ValueError("Las coordenadas deben ser números finitos")
        if isinstance(raw_value, bool) or raw_value not in (-1, 0, 1):
            raise ValueError("El valor de ocupación debe ser -1, 0 o 1")
        return x, y, int(raw_value)

    def detect(
        self,
        robot_pose: RobotPose,
        environment_matrix: Iterable[Sequence[object]],
    ) -> SensorMatrix:
        return self.scan(robot_pose, environment_matrix).detected

    def scan(
        self,
        robot_pose: RobotPose,
        environment_matrix: Iterable[Sequence[object]],
        *,
        angular_resolution: float = 2.0,
    ) -> SensorScan:
        """Detecta celdas y calcula el área visible mediante ray casting."""
        if not 0.1 <= angular_resolution <= 30.0:
            raise ValueError("angular_resolution debe estar entre 0.1 y 30 grados")
        robot_x, robot_y, robot_heading = self._pose_values(robot_pose)
        geometry = GridGeometry(self.grid_size)
        validated = [self._cell_values(cell) for cell in environment_matrix]
        occupied = {
            geometry.world_to_cell(x, y)
            for x, y, occupancy in validated
            if occupancy == 1
        }
        detected: SensorMatrix = []
        for x, y, occupancy in validated:
            dx, dy = x - robot_x, y - robot_y
            if hypot(dx, dy) > self.detection_radius:
                continue
            if self.field_of_view < 360.0:
                point_heading = degrees(atan2(dy, dx))
                angle_error = abs(
                    _angular_difference(point_heading, robot_heading)
                )
                if angle_error > self.field_of_view / 2.0:
                    continue
            if not self._has_line_of_sight(
                robot_x, robot_y, x, y, occupied
            ):
                continue
            detected.append([[x, y], occupancy])

        polygon = self._visibility_polygon(
            robot_x,
            robot_y,
            robot_heading,
            occupied,
            angular_resolution,
        )
        return SensorScan(detected, polygon)

    def _has_line_of_sight(
        self,
        robot_x: float,
        robot_y: float,
        target_x: float,
        target_y: float,
        occupied: set[GridCell],
    ) -> bool:
        distance = hypot(target_x - robot_x, target_y - robot_y)
        steps = max(1, ceil(distance / self.grid_size * 20.0))
        geometry = GridGeometry(self.grid_size)
        target_cell = geometry.world_to_cell(target_x, target_y)
        for index in range(1, steps):
            ratio = index / steps
            cell = geometry.world_to_cell(
                robot_x + (target_x - robot_x) * ratio,
                robot_y + (target_y - robot_y) * ratio,
            )
            if cell in occupied and cell != target_cell:
                return False
        return True

    def _visibility_polygon(
        self,
        robot_x: float,
        robot_y: float,
        robot_heading: float,
        occupied: set[GridCell],
        angular_resolution: float,
    ) -> tuple[tuple[float, float], ...]:
        ray_count = max(1, ceil(self.field_of_view / angular_resolution))
        if self.field_of_view >= 360.0:
            angles = [
                robot_heading + index * 360.0 / ray_count
                for index in range(ray_count)
            ]
            prefix: list[tuple[float, float]] = []
        else:
            angles = [
                robot_heading
                - self.field_of_view / 2.0
                + index * self.field_of_view / ray_count
                for index in range(ray_count + 1)
            ]
            prefix = [(robot_x, robot_y)]

        endpoints = [
            self._cast_ray(robot_x, robot_y, radians(angle), occupied)
            for angle in angles
        ]
        return tuple((*prefix, *endpoints))

    def _cast_ray(
        self,
        robot_x: float,
        robot_y: float,
        angle: float,
        occupied: set[GridCell],
    ) -> tuple[float, float]:
        step_size = self.grid_size / 25.0
        geometry = GridGeometry(self.grid_size)
        previous_distance = 0.0
        distance = step_size
        while distance <= self.detection_radius:
            x = robot_x + cos(angle) * distance
            y = robot_y + sin(angle) * distance
            if geometry.world_to_cell(x, y) in occupied:
                return (
                    robot_x + cos(angle) * previous_distance,
                    robot_y + sin(angle) * previous_distance,
                )
            previous_distance = distance
            distance += step_size
        return (
            robot_x + cos(angle) * self.detection_radius,
            robot_y + sin(angle) * self.detection_radius,
        )
