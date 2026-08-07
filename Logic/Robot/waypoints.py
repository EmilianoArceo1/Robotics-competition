"""Contenedor de coordenadas de una ruta."""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Sequence
from math import isfinite


class Waypoints:
    def __init__(self, coordinates: Iterable[Sequence[float]] = ()) -> None:
        self._coordinates: list[list[float]] = []
        self._current_index = 0
        self.replace(coordinates)

    @staticmethod
    def _validate(coordinate: Sequence[float]) -> list[float]:
        if len(coordinate) != 2:
            raise ValueError("Cada waypoint debe contener exactamente [x, y]")
        x, y = float(coordinate[0]), float(coordinate[1])
        if not isfinite(x) or not isfinite(y):
            raise ValueError("Las coordenadas deben ser números finitos")
        return [x, y]

    def replace(self, coordinates: Iterable[Sequence[float]]) -> None:
        self._coordinates = [self._validate(point) for point in coordinates]
        self._current_index = 0

    @property
    def matrix(self) -> list[list[float]]:
        return [point.copy() for point in self._coordinates]

    @property
    def current(self) -> tuple[float, float] | None:
        if self.complete:
            return None
        point = self._coordinates[self._current_index]
        return point[0], point[1]

    @property
    def current_index(self) -> int:
        return self._current_index

    @property
    def complete(self) -> bool:
        return self._current_index >= len(self._coordinates)

    def advance(self) -> bool:
        if not self.complete:
            self._current_index += 1
        return self.complete

    def reset(self) -> None:
        self._current_index = 0

    def __len__(self) -> int:
        return len(self._coordinates)

    def __iter__(self) -> Iterator[tuple[float, float]]:
        return ((point[0], point[1]) for point in self._coordinates)
