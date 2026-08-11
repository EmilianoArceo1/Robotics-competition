"""Selección de métodos intercambiables de clustering."""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module

ClusteringFactory = Callable[[], object]


def _connected_factory():
    module = import_module(
        "Logic.Methods.Objective Assign.Frontiers approaches."
        "Clustering methods.ConnectedComponents"
    )
    return module.ConnectedComponentsClustering()


class ClusteringController:
    NONE = "Sin clustering"

    def __init__(self) -> None:
        self._methods: dict[str, ClusteringFactory | None] = {
            self.NONE: None,
            "Componentes conectados": _connected_factory,
        }
        self._selected = self.NONE

    @property
    def available_methods(self) -> tuple[str, ...]:
        return tuple(self._methods)

    @property
    def selected_method(self) -> str:
        return self._selected

    @property
    def clustering_enabled(self) -> bool:
        return self._methods[self._selected] is not None

    def select(self, name: str) -> None:
        if name not in self._methods:
            raise ValueError(f"Método de clustering no registrado: {name}")
        self._selected = name

    def create(self):
        factory = self._methods[self._selected]
        return factory() if factory is not None else None

    def register(self, name: str, factory: ClusteringFactory) -> None:
        self._methods[name.strip()] = factory
