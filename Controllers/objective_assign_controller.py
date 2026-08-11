"""Selección de métodos de asignación de objetivos."""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module
from typing import Protocol


class ObjectiveMethod(Protocol):
    def assign_goal(self, belief_map: object, robot_state: object) -> list[float]: ...


ObjectiveFactory = Callable[[float, object | None], ObjectiveMethod]


def _nearest_frontier_factory(
    grid_size: float, clustering_method: object | None
) -> ObjectiveMethod:
    nearest_class = import_module(
        "Logic.Methods.Objective Assign.Frontiers approaches."
        "Frontier detection methods.NearestFrontier"
    ).NearestFrontier
    return nearest_class(
        cell_size=grid_size,
        clustering_method=clustering_method,
    )


class ObjectiveAssignController:
    def __init__(self) -> None:
        self._methods: dict[str, tuple[str, ObjectiveFactory | None]] = {
            "Frontera más cercana": (
                "nearest_frontier",
                _nearest_frontier_factory,
            )
        }
        self._selected = "Frontera más cercana"

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
        grid_size: float = 1.0,
        clustering_method: object | None = None,
    ) -> ObjectiveMethod:
        factory = self._methods[self._selected][1]
        if factory is None:
            raise NotImplementedError(
                f"{self._selected} todavía no tiene implementación"
            )
        return factory(grid_size, clustering_method)

    def select(self, method_name: str) -> None:
        if method_name not in self._methods:
            raise ValueError(
                f"Método de asignación no registrado: {method_name}"
            )
        self._selected = method_name

    def register(
        self,
        display_name: str,
        identifier: str,
        factory: ObjectiveFactory | None = None,
    ) -> None:
        name, method_id = display_name.strip(), identifier.strip()
        if not name or not method_id:
            raise ValueError("El nombre y el identificador son obligatorios")
        self._methods[name] = method_id, factory
