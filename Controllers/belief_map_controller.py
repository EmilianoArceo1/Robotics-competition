"""Adaptador visual para el mapa de creencias."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from Logic.Map.maps import Coordinate, SimulationMap
from Logic.Robot.BeliefMap import BeliefMap
from Logic.Map.grid_geometry import GridGeometry


@dataclass(frozen=True, slots=True)
class BeliefCell:
    coordinate: Coordinate
    occupancy: int


class BeliefMapController:
    DEFAULT_COLORS = {
        -1: "#d1d5db",
        0: "#f8fafc",
        1: "#475569",
    }

    def __init__(self, simulation_map: SimulationMap) -> None:
        self._map = simulation_map
        self._cells: tuple[BeliefCell, ...] = ()
        self._colors = self.DEFAULT_COLORS.copy()
        self._grid_size = 1.0

    @property
    def cells(self) -> tuple[BeliefCell, ...]:
        return self._cells

    @property
    def colors(self) -> dict[int, str]:
        return self._colors.copy()

    @property
    def grid_size(self) -> float:
        return self._grid_size

    @property
    def geometry(self) -> GridGeometry:
        return GridGeometry(self._grid_size)

    def configure_grid_size(self, grid_size: float) -> None:
        value = float(grid_size)
        if not isfinite(value) or not 0.1 <= value <= 5.0:
            raise ValueError("El tamaño del grid debe estar entre 0.1 y 5.0")
        self._grid_size = value

    def color_for(self, occupancy: int) -> str:
        if occupancy not in (-1, 0, 1):
            raise ValueError("La ocupación debe ser -1, 0 o 1")
        return self._colors[occupancy]

    def set_color(self, occupancy: int, color: str) -> None:
        if occupancy not in (-1, 0, 1):
            raise ValueError("La ocupación debe ser -1, 0 o 1")
        if not self._is_hex_color(color):
            raise ValueError("El color debe usar el formato hexadecimal #RRGGBB")
        self._colors[occupancy] = color.lower()

    def update(
        self,
        belief_map: BeliefMap,
        environment_matrix: list[list[object]],
    ) -> None:
        """Combina verdad-terreno y observaciones para representar desconocidos."""
        known = {
            (float(cell[0][0]), float(cell[0][1])): int(cell[1])
            for cell in belief_map.matrix
        }
        cells: list[BeliefCell] = []
        for cell in environment_matrix:
            local = float(cell[0][0]), float(cell[0][1])
            cells.append(
                BeliefCell(
                    coordinate=self._map.local_to_world(local),
                    occupancy=known.get(local, -1),
                )
            )
        self._cells = tuple(cells)

    def clear(self) -> None:
        self._cells = ()

    def replace_map(self, simulation_map: SimulationMap) -> None:
        self._map = simulation_map
        self.clear()

    @staticmethod
    def _is_hex_color(color: str) -> bool:
        if len(color) != 7 or not color.startswith("#"):
            return False
        return all(character in "0123456789abcdefABCDEF" for character in color[1:])
