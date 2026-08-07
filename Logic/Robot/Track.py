"""Planificación abstracta y seguimiento de rutas para el robot."""

from __future__ import annotations

from abc import ABC, abstractmethod
from math import atan2, hypot, pi
from typing import Sequence

from .Physic import RobotPhysics
from .waypoints import Waypoints

Coordinate = Sequence[float]
CoordinateMatrix = list[list[float]]


def _wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


class Track(ABC):
    def __init__(
        self,
        robot: RobotPhysics,
        *,
        waypoint_tolerance: float = 0.10,
        target_speed: float = 1.0,
        linear_gain: float = 1.5,
        angular_gain: float = 4.0,
        angular_damping: float = 1.5,
    ) -> None:
        if not isinstance(robot, RobotPhysics):
            raise TypeError("robot debe ser una instancia de RobotPhysics")
        if waypoint_tolerance <= 0.0:
            raise ValueError("waypoint_tolerance debe ser mayor que cero")
        if target_speed <= 0.0:
            raise ValueError("target_speed debe ser mayor que cero")
        self.robot = robot
        self.waypoints = Waypoints()
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.target_speed = float(target_speed)
        self.linear_gain = float(linear_gain)
        self.angular_gain = float(angular_gain)
        self.angular_damping = float(angular_damping)

    @abstractmethod
    def plan_route(
        self, start: Coordinate, goal: Coordinate
    ) -> CoordinateMatrix:
        """Calcula y devuelve una matriz de coordenadas [[x, y], ...]."""
        raise NotImplementedError

    def create_route(self, start: Coordinate, goal: Coordinate) -> Waypoints:
        self.waypoints.replace(self.plan_route(start, goal))
        return self.waypoints

    def compute_control(self) -> tuple[float, float] | None:
        """Calcula el control nominal hacia el waypoint activo.

        Devuelve ``None`` cuando la ruta ha terminado. Este método no modifica
        las físicas; el filtro SafeTracker se ejecuta antes de ``apply_control``.
        """
        target = self.waypoints.current
        if target is None:
            return None

        state = self.robot.state
        dx, dy = target[0] - state.x, target[1] - state.y
        distance = hypot(dx, dy)
        if distance <= self.waypoint_tolerance:
            self.waypoints.advance()
            target = self.waypoints.current
            if target is None:
                return None
            dx, dy = target[0] - state.x, target[1] - state.y
            distance = hypot(dx, dy)

        desired_heading = atan2(dy, dx)
        heading_error = _wrap_angle(desired_heading - state.theta)
        alignment = max(0.0, 1.0 - abs(heading_error) / (pi / 2.0))
        desired_speed = min(self.target_speed, distance) * alignment
        linear_acceleration = self.linear_gain * (
            desired_speed - state.linear_velocity
        )
        angular_acceleration = (
            self.angular_gain * heading_error
            - self.angular_damping * state.angular_velocity
        )
        return linear_acceleration, angular_acceleration

    def follow_waypoint(self) -> bool:
        """Compatibilidad para seguimiento sin filtro de seguridad."""
        nominal_control = self.compute_control()
        if nominal_control is None:
            self.robot.stop()
            return True
        self.robot.apply_control(*nominal_control)
        return False

    @property
    def route_complete(self) -> bool:
        return self.waypoints.complete
