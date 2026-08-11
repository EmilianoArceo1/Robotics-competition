"""Filtro HOCBF para evitar obstáculos estáticos observados."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, sin

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotState
from Logic.Robot.Sensor import SensorScan
from .SafeTracker import ControlCommand, SafeTracker


@dataclass(frozen=True, slots=True)
class CBFStatus:
    active: bool = False
    emergency: bool = False
    constraint_count: int = 0
    minimum_clearance: float | None = None
    reason: str = "no observations"


class CBFSafeTracker(SafeTracker):
    """HOCBF de segundo orden para un robot uniciclo dinámico.

    Para ``h = distancia - radio_seguro`` impone
    ``h_ddot + k1*h_dot + k0*h >= 0`` y proyecta la aceleración
    nominal sobre el intervalo factible. La aceleración angular conserva
    su valor nominal porque no aparece en la derivada de segundo orden.
    """

    def __init__(
        self,
        *,
        robot_radius: float = 0.305,
        safety_radius: float = 0.20,
        obstacle_radius: float = 0.50,
        max_acceleration: float = 2.0,
        k0: float = 2.0,
        k1: float = 3.0,
        influence_distance: float = 3.0,
    ) -> None:
        self.robot_radius = float(robot_radius)
        self.obstacle_radius = float(obstacle_radius)
        self.max_acceleration = float(max_acceleration)
        self.k0, self.k1 = float(k0), float(k1)
        self.influence_distance = float(influence_distance)
        self.configure_safety_radius(safety_radius)
        self.status = CBFStatus()

    @property
    def clearance_radius(self) -> float:
        return self.robot_radius + self.safety_radius + self.obstacle_radius

    @property
    def intervening(self) -> bool:
        return self.status.active

    def configure_safety_radius(self, radius: float) -> None:
        value = float(radius)
        if not isfinite(value) or not 0.0 <= value <= 5.0:
            raise ValueError("El radio de seguridad debe estar entre 0 y 5 m")
        self.safety_radius = value

    def filter_control(
        self,
        robot_state: RobotState,
        nominal_control: ControlCommand,
        belief_map: BeliefMap,
        sensor_scan: SensorScan | None,
        dt: float,
    ) -> ControlCommand:
        if dt <= 0.0:
            raise ValueError("dt debe ser mayor que cero")
        nominal_a, nominal_alpha = map(float, nominal_control)
        if sensor_scan is None:
            self.status = CBFStatus()
            return nominal_a, nominal_alpha

        heading = cos(robot_state.theta), sin(robot_state.theta)
        perpendicular = -heading[1], heading[0]
        speed = float(robot_state.linear_velocity)
        omega = float(robot_state.angular_velocity)
        lower, upper = -self.max_acceleration, self.max_acceleration
        constraint_count = 0
        minimum_clearance: float | None = None

        for cell in sensor_scan.detected:
            if int(cell[1]) != 1:
                continue
            obstacle_x, obstacle_y = float(cell[0][0]), float(cell[0][1])
            dx = robot_state.x - obstacle_x
            dy = robot_state.y - obstacle_y
            distance = hypot(dx, dy)
            clearance = distance - self.clearance_radius
            minimum_clearance = (
                clearance if minimum_clearance is None
                else min(minimum_clearance, clearance)
            )
            if clearance > self.influence_distance:
                continue
            constraint_count += 1
            if distance <= 1e-9 or clearance <= 0.0:
                self.status = CBFStatus(
                    True, True, constraint_count, minimum_clearance,
                    "safety envelope violated",
                )
                return -self.max_acceleration, 0.0

            normal = dx / distance, dy / distance
            radial_speed = speed * (
                normal[0] * heading[0] + normal[1] * heading[1]
            )
            tangential_speed_sq = max(0.0, speed * speed - radial_speed * radial_speed)
            drift = (
                speed * omega
                * (normal[0] * perpendicular[0] + normal[1] * perpendicular[1])
                + tangential_speed_sq / distance
                + self.k1 * radial_speed
                + self.k0 * clearance
            )
            coefficient = normal[0] * heading[0] + normal[1] * heading[1]
            if abs(coefficient) <= 1e-9:
                if drift < 0.0:
                    self.status = CBFStatus(
                        True, True, constraint_count, minimum_clearance,
                        "CBF constraint infeasible",
                    )
                    return -self.max_acceleration, 0.0
                continue
            boundary = -drift / coefficient
            if coefficient > 0.0:
                lower = max(lower, boundary)
            else:
                upper = min(upper, boundary)

        if lower > upper:
            self.status = CBFStatus(
                True, True, constraint_count, minimum_clearance,
                "CBF interval infeasible",
            )
            return -self.max_acceleration, 0.0
        safe_a = max(lower, min(nominal_a, upper))
        active = abs(safe_a - nominal_a) > 1e-9
        self.status = CBFStatus(
            active, False, constraint_count, minimum_clearance,
            "HOCBF projection" if active else "nominal control certified",
        )
        return safe_a, nominal_alpha
