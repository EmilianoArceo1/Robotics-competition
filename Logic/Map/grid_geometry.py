"""Convención única para la cuadrícula métrica del simulador.

Las coordenadas enteras identifican celdas y sus centros están en múltiplos
de ``resolution``. Por tanto, la celda ``(0, 0)`` ocupa desde
``-resolution / 2`` hasta ``+resolution / 2`` en ambos ejes.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import floor, isfinite


@dataclass(frozen=True, slots=True)
class GridCell:
    column: int
    row: int


@dataclass(frozen=True, slots=True)
class GridGeometry:
    resolution: float = 1.0

    def __post_init__(self) -> None:
        value = float(self.resolution)
        if not isfinite(value) or not 0.1 <= value <= 5.0:
            raise ValueError("resolution debe estar entre 0.1 y 5.0")
        object.__setattr__(self, "resolution", value)

    def world_to_cell(self, x: float, y: float) -> GridCell:
        """Devuelve la celda que contiene un punto continuo."""
        half = self.resolution / 2.0
        return GridCell(
            column=floor((float(x) + half) / self.resolution),
            row=floor((float(y) + half) / self.resolution),
        )

    def cell_to_world(self, cell: GridCell) -> tuple[float, float]:
        """Devuelve el centro métrico de una celda."""
        return (
            float(cell.column * self.resolution),
            float(cell.row * self.resolution),
        )

    def cell_bounds(self, cell: GridCell) -> tuple[float, float, float, float]:
        # Los vecinos calculan su borde compartido con la misma expresión;
        # evita diferencias como 0.75 frente a 0.7499999999999999.
        return (
            float((cell.column - 0.5) * self.resolution),
            float((cell.column + 0.5) * self.resolution),
            float((cell.row - 0.5) * self.resolution),
            float((cell.row + 0.5) * self.resolution),
        )

    @staticmethod
    def neighbors4(cell: GridCell) -> tuple[GridCell, ...]:
        return (
            GridCell(cell.column + 1, cell.row),
            GridCell(cell.column - 1, cell.row),
            GridCell(cell.column, cell.row + 1),
            GridCell(cell.column, cell.row - 1),
        )
