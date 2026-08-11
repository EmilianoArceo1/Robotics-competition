"""Geometría exacta y explicable para colisiones del robot planar."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from math import hypot, isfinite

Point = tuple[float, float]
Rectangle = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class CollisionReport:
    collision: bool
    reason: str = "clear"
    obstacle: Rectangle | None = None
    distance: float | None = None


def distance_point_to_segment(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return hypot(point[0] - start[0], point[1] - start[1])
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    closest = start[0] + ratio * dx, start[1] + ratio * dy
    return hypot(point[0] - closest[0], point[1] - closest[1])


def distance_point_to_rect(point: Point, rectangle: Rectangle) -> float:
    x, y, width, height = rectangle
    x_min, x_max = sorted((x, x + width))
    y_min, y_max = sorted((y, y + height))
    dx = max(x_min - point[0], 0.0, point[0] - x_max)
    dy = max(y_min - point[1], 0.0, point[1] - y_max)
    return hypot(dx, dy)


def _orientation(a: Point, b: Point, c: Point) -> int:
    value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if abs(value) <= 1e-12:
        return 0
    return 1 if value > 0.0 else 2


def _on_segment(a: Point, b: Point, c: Point) -> bool:
    return (
        min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
        and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])
    )


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    orientations = (
        _orientation(a, b, c),
        _orientation(a, b, d),
        _orientation(c, d, a),
        _orientation(c, d, b),
    )
    if orientations[0] != orientations[1] and orientations[2] != orientations[3]:
        return True
    return (
        (orientations[0] == 0 and _on_segment(a, c, b))
        or (orientations[1] == 0 and _on_segment(a, d, b))
        or (orientations[2] == 0 and _on_segment(c, a, d))
        or (orientations[3] == 0 and _on_segment(c, b, d))
    )


def _rect_edges(rectangle: Rectangle) -> tuple[tuple[Point, Point], ...]:
    x, y, width, height = rectangle
    corners = ((x, y), (x + width, y), (x + width, y + height), (x, y + height))
    return tuple((corners[index], corners[(index + 1) % 4]) for index in range(4))


def distance_segment_to_rect(start: Point, end: Point, rectangle: Rectangle) -> float:
    if distance_point_to_rect(start, rectangle) == 0.0 or distance_point_to_rect(end, rectangle) == 0.0:
        return 0.0
    edges = _rect_edges(rectangle)
    if any(_segments_intersect(start, end, edge[0], edge[1]) for edge in edges):
        return 0.0
    endpoint_distance = min(
        distance_point_to_rect(start, rectangle),
        distance_point_to_rect(end, rectangle),
    )
    corner_distance = min(
        distance_point_to_segment(edge[0], start, end) for edge in edges
    )
    return min(endpoint_distance, corner_distance)


class CollisionChecker:
    """Robot circular contra obstáculos cuadrados con esquinas redondeadas."""

    def __init__(
        self,
        obstacles: Iterable[Sequence[float]],
        *,
        robot_radius: float,
        safety_radius: float = 0.0,
        obstacle_size: float = 1.0,
    ) -> None:
        half = float(obstacle_size) / 2.0
        self.obstacles: tuple[Rectangle, ...] = tuple(
            (float(point[0]) - half, float(point[1]) - half, float(obstacle_size), float(obstacle_size))
            for point in obstacles
        )
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

    def check_position(self, position: Sequence[float]) -> CollisionReport:
        point = float(position[0]), float(position[1])
        for obstacle in self.obstacles:
            distance = distance_point_to_rect(point, obstacle)
            if distance <= self.clearance_radius + 1e-12:
                return CollisionReport(True, "position intersects safety envelope", obstacle, distance)
        return CollisionReport(False)

    def check_motion(self, start: Sequence[float], end: Sequence[float]) -> CollisionReport:
        start_point = float(start[0]), float(start[1])
        end_point = float(end[0]), float(end[1])
        for obstacle in self.obstacles:
            distance = distance_segment_to_rect(start_point, end_point, obstacle)
            if distance <= self.clearance_radius + 1e-12:
                return CollisionReport(True, "motion intersects safety envelope", obstacle, distance)
        return CollisionReport(False)

    def collides(self, position: Sequence[float]) -> bool:
        return self.check_position(position).collision

    def motion_collides(self, start: Sequence[float], end: Sequence[float]) -> bool:
        return self.check_motion(start, end).collision
