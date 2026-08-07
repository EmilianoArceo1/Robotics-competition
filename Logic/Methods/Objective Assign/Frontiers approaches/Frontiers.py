"""Detección abstracta de fronteras de exploración."""

from __future__ import annotations

from abc import ABC, abstractmethod

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Track import CoordinateMatrix
from Logic.Map.grid_geometry import GridGeometry


class Frontiers(ABC):
    def __init__(self, *, cell_size: float = 1.0) -> None:
        if cell_size <= 0.0:
            raise ValueError("cell_size debe ser mayor que cero")
        self.cell_size = float(cell_size)
        self.geometry = GridGeometry(self.cell_size)
        self.raw_frontiers: CoordinateMatrix = []

    def detect_frontiers(self, belief_map: BeliefMap) -> CoordinateMatrix:
        if not isinstance(belief_map, BeliefMap):
            raise TypeError("belief_map debe ser una instancia de BeliefMap")
        # La topología de la cuadrícula se expresa con índices enteros.
        # Las coordenadas métricas sólo se reconstruyen al devolver resultados.
        cells = {
            self.geometry.world_to_cell(
                float(cell[0][0]), float(cell[0][1])
            ): int(cell[1])
            for cell in belief_map.matrix
        }
        self.raw_frontiers = [
            [*self.geometry.cell_to_world(cell)]
            for cell, occupancy in cells.items()
            if occupancy == 0
            and any(
                cells.get(neighbor, -1) == -1
                for neighbor in self.geometry.neighbors4(cell)
            )
        ]
        return [coordinate.copy() for coordinate in self.raw_frontiers]

    @abstractmethod
    def cluster_frontiers(
        self, frontiers: CoordinateMatrix
    ) -> CoordinateMatrix:
        """Agrupa las fronteras y devuelve coordenadas candidatas."""
        raise NotImplementedError

    def get_frontiers(self, belief_map: BeliefMap) -> CoordinateMatrix:
        return self.cluster_frontiers(self.detect_frontiers(belief_map))


__all__ = ["Frontiers"]
