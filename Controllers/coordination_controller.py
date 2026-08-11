"""Selección de estrategias de coordinación intercambiables."""

from __future__ import annotations

from collections.abc import Callable

from Logic.Methods.Coordination import (
    CentralizedGreedyCoordinator, Coordinator, LocalAuctionCoordinator,
    NoCoordination,
)

CoordinatorFactory = Callable[[], Coordinator]


class CoordinationController:
    def __init__(self) -> None:
        self._methods: dict[str, CoordinatorFactory] = {
            "Sin coordinación": NoCoordination,
            "Centralizado greedy": CentralizedGreedyCoordinator,
            "Subasta descentralizada": LocalAuctionCoordinator,
        }
        self._selected = "Sin coordinación"

    @property
    def available_methods(self) -> tuple[str, ...]:
        return tuple(self._methods)

    @property
    def selected_method(self) -> str:
        return self._selected

    def select(self, name: str) -> None:
        if name not in self._methods:
            raise ValueError(f"Coordinador no registrado: {name}")
        self._selected = name

    def create(self) -> Coordinator:
        return self._methods[self._selected]()

    def register(self, name: str, factory: CoordinatorFactory) -> None:
        value = name.strip()
        if not value:
            raise ValueError("El nombre del coordinador es obligatorio")
        self._methods[value] = factory


__all__ = ["CoordinationController"]
