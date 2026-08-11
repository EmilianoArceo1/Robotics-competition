"""Simplificación de rutas conservando transitabilidad del costmap."""

from __future__ import annotations

from Logic.Map.grid_geometry import GridCell
from .costmap import PlanningCostmap


class PathSimplifier:
    def __init__(self, costmap: PlanningCostmap) -> None:
        self.costmap = costmap

    @staticmethod
    def segment_cells(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
        """Supercover: incluye toda celda tocada y ambos lados de una esquina."""
        x, y = start.column, start.row
        dx, dy = end.column - x, end.row - y
        nx, ny = abs(dx), abs(dy)
        sign_x = 1 if dx > 0 else -1 if dx < 0 else 0
        sign_y = 1 if dy > 0 else -1 if dy < 0 else 0
        cells: list[GridCell] = [start]
        ix = iy = 0
        while ix < nx or iy < ny:
            decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
            if decision == 0:
                # El segmento cruza una esquina: ambos laterales deben ser libres.
                if sign_x:
                    cells.append(GridCell(x + sign_x, y))
                if sign_y:
                    cells.append(GridCell(x, y + sign_y))
                x += sign_x
                y += sign_y
                ix += 1
                iy += 1
            elif decision < 0:
                x += sign_x
                ix += 1
            else:
                y += sign_y
                iy += 1
            cells.append(GridCell(x, y))
        return tuple(dict.fromkeys(cells))

    def segment_is_safe(self, start: GridCell, end: GridCell) -> bool:
        return all(
            self.costmap.is_traversable(cell)
            for cell in self.segment_cells(start, end)
        )

    def simplify(self, path: list[GridCell]) -> list[GridCell]:
        if len(path) <= 2:
            return path.copy()
        simplified = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            candidate = len(path) - 1
            while candidate > anchor + 1:
                if self.segment_is_safe(path[anchor], path[candidate]):
                    break
                candidate -= 1
            simplified.append(path[candidate])
            anchor = candidate
        return simplified
