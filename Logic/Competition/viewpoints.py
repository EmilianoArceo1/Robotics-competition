"""Generación de viewpoints usando únicamente el mapa observado."""

from __future__ import annotations

from abc import ABC, abstractmethod
from math import cos, pi, sin
import numpy as np


class ViewpointGenerator(ABC):
    @abstractmethod
    def generate(self, observed_map: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """Devuelve posiciones known-free candidatas en formato (row, col)."""


class FrontierOcclusionViewpointGenerator(ViewpointGenerator):
    def __init__(self, *, offsets=(0, 5, 10, 15, 20), angles=16,
                 maximum_candidates=160) -> None:
        self.offsets = tuple(int(value) for value in offsets)
        self.angles = int(angles)
        self.maximum_candidates = int(maximum_candidates)

    @staticmethod
    def _frontier_cells(observed: np.ndarray) -> np.ndarray:
        unknown = observed == .5
        adjacent_unknown = np.zeros_like(unknown)
        adjacent_unknown[1:] |= unknown[:-1]; adjacent_unknown[:-1] |= unknown[1:]
        adjacent_unknown[:,1:] |= unknown[:,:-1]; adjacent_unknown[:,:-1] |= unknown[:,1:]
        return np.argwhere((observed == 0) & adjacent_unknown)

    def generate(self, observed_map: np.ndarray, pose: np.ndarray) -> np.ndarray:
        frontiers = self._frontier_cells(observed_map)
        if not len(frontiers):
            return np.empty((0, 2), dtype=int)
        # Muestreo espacial determinista: conserva cobertura de fronteras largas.
        stride = max(1, len(frontiers) // 48)
        seeds = frontiers[::stride]
        candidates: set[tuple[int, int]] = set()
        for seed in seeds:
            for radius in self.offsets:
                for angle in np.linspace(0, 2*pi, self.angles, endpoint=False):
                    point = np.rint(seed + (sin(angle)*radius, cos(angle)*radius)).astype(int)
                    if (0 <= point[0] < observed_map.shape[0] and
                            0 <= point[1] < observed_map.shape[1] and
                            observed_map[tuple(point)] == 0):
                        candidates.add(tuple(map(int, point)))

        # Occlusion viewpoints: known-free junto a pared y cerca de unknown.
        occupied = observed_map == 1
        near_wall = np.zeros_like(occupied)
        near_wall[1:] |= occupied[:-1]; near_wall[:-1] |= occupied[1:]
        near_wall[:,1:] |= occupied[:,:-1]; near_wall[:,:-1] |= occupied[:,1:]
        wall_candidates = np.argwhere((observed_map == 0) & near_wall)
        if len(wall_candidates):
            # Prioriza los próximos a alguna frontera, típico de puertas/esquinas.
            sampled = wall_candidates[::max(1, len(wall_candidates)//80)]
            for point in sampled:
                if np.linalg.norm(frontiers - point, axis=1).min() <= 25:
                    candidates.add(tuple(map(int, point)))
        ordered = sorted(candidates, key=lambda point: np.linalg.norm(np.asarray(point)-pose))
        return np.asarray(ordered[:self.maximum_candidates], dtype=int).reshape(-1, 2)
