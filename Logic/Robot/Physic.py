"""Modelo cinemático y representación física de un robot móvil planar."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, pi, sin


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


@dataclass(frozen=True, slots=True)
class RobotGeometry:
    length: float = 0.50
    width: float = 0.35

    def __post_init__(self) -> None:
        if self.length <= 0.0 or self.width <= 0.0:
            raise ValueError("Las dimensiones del robot deben ser positivas")


@dataclass(frozen=True, slots=True)
class ControlLimits:
    max_linear_acceleration: float = 2.0
    max_angular_acceleration: float = 4.0
    max_linear_speed: float = 3.0
    max_angular_speed: float = 6.0

    def __post_init__(self) -> None:
        if min(
            self.max_linear_acceleration,
            self.max_angular_acceleration,
            self.max_linear_speed,
            self.max_angular_speed,
        ) <= 0.0:
            raise ValueError("Todos los límites físicos deben ser positivos")


@dataclass(slots=True)
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    linear_acceleration: float = 0.0
    angular_acceleration: float = 0.0


@dataclass(slots=True)
class RobotPhysics:
    """Robot tipo uniciclo controlado por aceleraciones."""

    geometry: RobotGeometry = field(default_factory=RobotGeometry)
    limits: ControlLimits = field(default_factory=ControlLimits)
    state: RobotState = field(default_factory=RobotState)

    def apply_control(
        self, linear_acceleration: float, angular_acceleration: float
    ) -> None:
        self.state.linear_acceleration = _clamp(
            float(linear_acceleration),
            -self.limits.max_linear_acceleration,
            self.limits.max_linear_acceleration,
        )
        self.state.angular_acceleration = _clamp(
            float(angular_acceleration),
            -self.limits.max_angular_acceleration,
            self.limits.max_angular_acceleration,
        )

    def step(self, dt: float) -> RobotState:
        if dt <= 0.0:
            raise ValueError("dt debe ser mayor que cero")

        state = self.state
        old_linear_velocity = state.linear_velocity
        old_angular_velocity = state.angular_velocity
        state.linear_velocity = _clamp(
            old_linear_velocity + state.linear_acceleration * dt,
            -self.limits.max_linear_speed,
            self.limits.max_linear_speed,
        )
        state.angular_velocity = _clamp(
            old_angular_velocity + state.angular_acceleration * dt,
            -self.limits.max_angular_speed,
            self.limits.max_angular_speed,
        )

        mean_linear_velocity = 0.5 * (
            old_linear_velocity + state.linear_velocity
        )
        mean_angular_velocity = 0.5 * (
            old_angular_velocity + state.angular_velocity
        )
        mean_theta = state.theta + 0.5 * mean_angular_velocity * dt
        state.x += mean_linear_velocity * cos(mean_theta) * dt
        state.y += mean_linear_velocity * sin(mean_theta) * dt
        state.theta = _wrap_angle(
            state.theta + mean_angular_velocity * dt
        )
        return state

    def footprint(self) -> tuple[tuple[float, float], ...]:
        half_length = self.geometry.length / 2.0
        half_width = self.geometry.width / 2.0
        local_corners = (
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        )
        cosine, sine = cos(self.state.theta), sin(self.state.theta)
        return tuple(
            (
                self.state.x + local_x * cosine - local_y * sine,
                self.state.y + local_x * sine + local_y * cosine,
            )
            for local_x, local_y in local_corners
        )

    def stop(self) -> None:
        self.state.linear_velocity = 0.0
        self.state.angular_velocity = 0.0
        self.apply_control(0.0, 0.0)
