"""Contrato intercambiable para agrupar celdas frontera."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from Logic.Robot.Track import CoordinateMatrix


@dataclass(frozen=True, slots=True)
class FrontierCluster:
    identifier: int
    cells: tuple[tuple[float, float], ...]
    centroid: tuple[float, float]
    representative: tuple[float, float]

    @property
    def size(self) -> int:
        return len(self.cells)


class ClusteringMethod(ABC):
    @abstractmethod
    def cluster(
        self,
        frontiers: CoordinateMatrix,
        *,
        cell_size: float,
    ) -> list[FrontierCluster]:
        """Agrupa fronteras y devuelve clusters con representante."""
        raise NotImplementedError
