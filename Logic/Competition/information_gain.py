"""Estrategias intercambiables para estimar information gain."""

from __future__ import annotations

from abc import ABC, abstractmethod
import numpy as np


class InformationGainMethod(ABC):
    name = "Abstract"

    @abstractmethod
    def calculate(self, observed_map: np.ndarray, candidate: np.ndarray) -> float:
        """Devuelve una ganancia no normalizada para una meta candidata."""


class CircularUnknownInformationGain(InformationGainMethod):
    name = "Unknown cells (circular)"

    def __init__(self, radius: int = 25) -> None:
        if radius <= 0:
            raise ValueError("El radio de information gain debe ser positivo")
        self.radius = int(radius)

    def calculate(self, observed_map: np.ndarray, candidate: np.ndarray) -> float:
        row, col = map(int, candidate)
        radius = self.radius
        y0, y1 = max(0, row-radius), min(observed_map.shape[0], row+radius+1)
        x0, x1 = max(0, col-radius), min(observed_map.shape[1], col+radius+1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        circle = (yy-row)**2 + (xx-col)**2 <= radius**2
        return float(np.count_nonzero((observed_map[y0:y1, x0:x1] == 0.5) & circle))


class FrontierDensityInformationGain(InformationGainMethod):
    name = "Frontier density"

    def __init__(self, radius: int = 25) -> None:
        self.radius = int(radius)

    def calculate(self, observed_map: np.ndarray, candidate: np.ndarray) -> float:
        row, col = map(int, candidate); radius = self.radius
        y0, y1 = max(1, row-radius), min(observed_map.shape[0]-1, row+radius+1)
        x0, x1 = max(1, col-radius), min(observed_map.shape[1]-1, col+radius+1)
        area = observed_map[y0:y1, x0:x1]
        unknown = area == 0.5
        adjacent_free = np.zeros_like(unknown)
        free = area == 0
        adjacent_free[1:] |= free[:-1]; adjacent_free[:-1] |= free[1:]
        adjacent_free[:,1:] |= free[:,:-1]; adjacent_free[:,:-1] |= free[:,1:]
        return float(np.count_nonzero(unknown & adjacent_free))


class PotentialVisibilityInformationGain(InformationGainMethod):
    """Unknown potencialmente visible mediante raycasting sobre lo conocido."""

    name = "Potential visibility (raycast)"

    def __init__(self, radius: int = 100, rays: int = 180) -> None:
        self.radius = int(radius)
        self.rays = int(rays)
        if self.radius <= 0 or self.rays < 8:
            raise ValueError("Potential visibility requiere radio > 0 y al menos 8 rayos")

    def calculate(self, observed_map: np.ndarray, candidate: np.ndarray) -> float:
        from math import cos, pi, sin
        origin = np.asarray(candidate, dtype=int)
        visible_unknown: set[tuple[int, int]] = set()
        for angle in np.linspace(0, 2*pi, self.rays, endpoint=False):
            end = origin + np.array((sin(angle), cos(angle))) * self.radius
            count = max(abs(int(end[0])-int(origin[0])),
                        abs(int(end[1])-int(origin[1]))) + 1
            rows = np.rint(np.linspace(origin[0], end[0], count)).astype(int)
            cols = np.rint(np.linspace(origin[1], end[1], count)).astype(int)
            valid = ((rows >= 0) & (cols >= 0) &
                     (rows < observed_map.shape[0]) & (cols < observed_map.shape[1]))
            for row, col in zip(rows[valid], cols[valid]):
                value = observed_map[row, col]
                if value == 1:
                    break
                if value == 0.5:
                    visible_unknown.add((int(row), int(col)))
        return float(len(visible_unknown))


METHODS = {method.name: method for method in
           (CircularUnknownInformationGain, FrontierDensityInformationGain,
            PotentialVisibilityInformationGain)}


def create_information_gain(name: str, radius: int = 25) -> InformationGainMethod:
    try:
        return METHODS[name](radius)
    except KeyError as error:
        raise ValueError(f"Método de information gain desconocido: {name}") from error
