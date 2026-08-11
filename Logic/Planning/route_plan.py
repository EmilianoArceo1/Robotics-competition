"""Resultado explícito de una solicitud de planificación."""

from __future__ import annotations

from dataclasses import dataclass

Coordinate = tuple[float, float]


@dataclass(frozen=True, slots=True)
class RoutePlanResult:
    success: bool
    reason: str
    requested_goal: Coordinate
    safe_goal: Coordinate | None
    raw_path: tuple[Coordinate, ...] = ()
    simplified_path: tuple[Coordinate, ...] = ()
    evaluated_cells: int = 0

