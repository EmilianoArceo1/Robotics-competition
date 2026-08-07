"""Mapa acumulativo construido a partir de observaciones del sensor."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from math import isfinite

from .Sensor import SensorMatrix


class BeliefMap:
    def __init__(
        self, initial_observations: Iterable[Sequence[object]] = ()
    ) -> None:
        self._cells: dict[tuple[float, float], int] = {}
        self.update(initial_observations)

    @staticmethod
    def _validate_cell(cell: Sequence[object]) -> tuple[float, float, int]:
        if len(cell) != 2:
            raise ValueError("Cada celda debe tener la forma [[x, y], valor]")
        coordinate, raw_value = cell
        if not isinstance(coordinate, Sequence) or isinstance(
            coordinate, (str, bytes)
        ):
            raise ValueError("La coordenada debe tener la forma [x, y]")
        if len(coordinate) != 2:
            raise ValueError("La coordenada debe contener exactamente [x, y]")
        x, y = float(coordinate[0]), float(coordinate[1])
        if not isfinite(x) or not isfinite(y):
            raise ValueError("Las coordenadas deben ser números finitos")
        if isinstance(raw_value, bool) or raw_value not in (-1, 0, 1):
            raise ValueError("El valor de ocupación debe ser -1, 0 o 1")
        return x, y, int(raw_value)

    def update(self, observations: Iterable[Sequence[object]]) -> None:
        validated = [self._validate_cell(cell) for cell in observations]
        for x, y, occupancy in validated:
            self._cells[(x, y)] = occupancy

    @property
    def matrix(self) -> SensorMatrix:
        return [
            [[x, y], occupancy]
            for (x, y), occupancy in self._cells.items()
        ]

    def value_at(self, coordinate: Sequence[float]) -> int | None:
        if len(coordinate) != 2:
            raise ValueError("La coordenada debe contener exactamente [x, y]")
        return self._cells.get((float(coordinate[0]), float(coordinate[1])))

    def clear(self) -> None:
        self._cells.clear()

    def __len__(self) -> int:
        return len(self._cells)
