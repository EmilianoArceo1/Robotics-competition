"""Drop-in mínimo de ``pyastar2d`` para ejecutar policies oficiales."""
from Logic.Competition.environment import astar


def astar_path(weights, start, goal, allow_diagonal=False):
    if allow_diagonal:
        raise ValueError("El entorno de competición usa allow_diagonal=False")
    return astar(weights, start, goal)


__all__ = ["astar_path"]
