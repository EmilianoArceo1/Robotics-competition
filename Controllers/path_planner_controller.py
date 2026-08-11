"""Selección de algoritmos de planificación de rutas."""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from Logic.Robot.Physic import RobotPhysics
    from Logic.Robot.Track import Track

PlannerFactory = Callable[
    ["RobotPhysics", tuple[tuple[float, float], ...], float, float], "Track"
]


def _astar_factory(
    robot: "RobotPhysics",
    obstacles: tuple[tuple[float, float], ...],
    grid_size: float,
    safety_radius: float,
) -> "Track":
    astar_class = import_module(
        "Logic.Methods.Path planers.Astar"
    ).AStar
    return astar_class(
        robot,
        obstacles,
        grid_size=grid_size,
        safety_radius=safety_radius,
    )


class PathPlannerController:
    def __init__(self) -> None:
        self._methods: dict[str, tuple[str, PlannerFactory | None]] = {
            "A*": ("astar", _astar_factory)
        }
        self._selected = "A*"

    @property
    def available_methods(self) -> tuple[str, ...]:
        return tuple(self._methods)

    @property
    def selected_method(self) -> str:
        return self._selected

    @property
    def selected_identifier(self) -> str:
        return self._methods[self._selected][0]

    def create(
        self,
        robot: "RobotPhysics",
        obstacles: tuple[tuple[float, float], ...],
        grid_size: float = 1.0,
        safety_radius: float = 0.0,
    ) -> "Track":
        factory = self._methods[self._selected][1]
        if factory is None:
            raise NotImplementedError(
                f"{self._selected} todavía no tiene implementación"
            )
        return factory(robot, obstacles, grid_size, safety_radius)

    def select(self, method_name: str) -> None:
        if method_name not in self._methods:
            raise ValueError(f"Planificador no registrado: {method_name}")
        self._selected = method_name

    def register(
        self,
        display_name: str,
        identifier: str,
        factory: PlannerFactory | None = None,
    ) -> None:
        name, method_id = display_name.strip(), identifier.strip()
        if not name or not method_id:
            raise ValueError("El nombre y el identificador son obligatorios")
        self._methods[name] = method_id, factory
