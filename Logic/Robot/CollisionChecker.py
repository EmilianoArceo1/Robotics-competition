"""Comprobación geométrica de colisiones para el robot planar."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from math import ceil, hypot, isfinite


class CollisionChecker:
    """Comprueba un robot circular contra obstáculos cuadrados del mapa.

    ``robot_radius`` es el radio de la huella física y ``safety_radius`` es
    el margen adicional que debe permanecer libre alrededor de ella.
    """

    def __init__(
        self,
        obstacles: Iterable[Sequence[float]],
        *,
        robot_radius: float,
        safety_radius: float = 0.0,
        obstacle_size: float = 1.0,
    ) -> None:
        self.obstacles = tuple((float(point[0]), float(point[1])) for point in obstacles)
        self.robot_radius = float(robot_radius)
        self.obstacle_size = float(obstacle_size)
        if self.robot_radius <= 0.0 or self.obstacle_size <= 0.0:
            raise ValueError("Los tamaños geométricos deben ser positivos")
        self.configure_safety_radius(safety_radius)

    @property
    def clearance_radius(self) -> float:
        return self.robot_radius + self.safety_radius

    def configure_safety_radius(self, radius: float) -> None:
        value = float(radius)
        if not isfinite(value) or not 0.0 <= value <= 5.0:
            raise ValueError("El radio de seguridad debe estar entre 0 y 5 m")
        self.safety_radius = value

    def collides(self, position: Sequence[float]) -> bool:
        x, y = float(position[0]), float(position[1])
        half = self.obstacle_size / 2.0
        radius = self.clearance_radius
        for obstacle_x, obstacle_y in self.obstacles:
            nearest_x = max(obstacle_x - half, min(x, obstacle_x + half))
            nearest_y = max(obstacle_y - half, min(y, obstacle_y + half))
            if hypot(x - nearest_x, y - nearest_y) <= radius:
                return True
        return False

    def motion_collides(
        self,
        start: Sequence[float],
        end: Sequence[float],
    ) -> bool:
        distance = hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        step = max(0.02, self.clearance_radius / 3.0)
        samples = max(1, ceil(distance / step))
        return any(
            self.collides(
                (
                    float(start[0]) + (float(end[0]) - float(start[0])) * index / samples,
                    float(start[1]) + (float(end[1]) - float(start[1])) * index / samples,
                )
            )
            for index in range(1, samples + 1)
        )
