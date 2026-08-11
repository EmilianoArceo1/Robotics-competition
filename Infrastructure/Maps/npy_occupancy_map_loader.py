"""Adaptador para occupancy grids NPY de la Indoor Exploration Competition."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from Logic.Map.maps import SimulationMap


class NpyOccupancyMapLoader:
    def load(
        self,
        source: str,
        *,
        start_pose: tuple[float, float] = (15.0, 15.0),
        source_resolution: float = 0.05,
        target_resolution: float = 0.5,
        minimum_start_clearance: float = 1.6,
    ) -> SimulationMap:
        if source_resolution <= 0.0 or target_resolution < source_resolution:
            raise ValueError("Las resoluciones del mapa no son válidas")
        factor = round(target_resolution / source_resolution)
        if abs(factor * source_resolution - target_resolution) > 1e-9:
            raise ValueError("target_resolution debe ser múltiplo de source_resolution")

        raw = np.load(Path(source), allow_pickle=False)
        if raw.ndim != 2:
            raise ValueError("occ_map.npy debe ser una matriz bidimensional")
        values = set(int(value) for value in np.unique(raw))
        if not values.issubset({0, 254, 255}):
            raise ValueError(f"Valores de occupancy grid no soportados: {values}")

        rows = (raw.shape[0] + factor - 1) // factor
        columns = (raw.shape[1] + factor - 1) // factor
        padded = np.pad(
            raw,
            ((0, rows * factor - raw.shape[0]),
             (0, columns * factor - raw.shape[1])),
            constant_values=0,
        )
        reduced = padded.reshape(rows, factor, columns, factor).min(axis=(1, 3))
        occupied_rows, occupied_columns = np.where(reduced == 0)
        desired_x = float(start_pose[0]) * source_resolution
        desired_y = float(start_pose[1]) * source_resolution
        origin_x, origin_y = desired_x, desired_y
        if minimum_start_clearance > 0.0:
            free_rows, free_columns = np.where(reduced != 0)
            free_x = (free_columns.astype(float) + 0.5) * target_resolution
            free_y = (free_rows.astype(float) + 0.5) * target_resolution
            order = np.argsort((free_x - desired_x) ** 2 + (free_y - desired_y) ** 2)
            occupied_x = (occupied_columns.astype(float) + 0.5) * target_resolution
            occupied_y = (occupied_rows.astype(float) + 0.5) * target_resolution
            required_center_distance = (
                float(minimum_start_clearance) + target_resolution / 2.0
            )
            required_squared = required_center_distance ** 2
            for index in order:
                candidate_x, candidate_y = free_x[index], free_y[index]
                distances = (
                    (occupied_x - candidate_x) ** 2
                    + (occupied_y - candidate_y) ** 2
                )
                if distances.size == 0 or float(distances.min()) >= required_squared:
                    origin_x, origin_y = candidate_x, candidate_y
                    break
            else:
                raise ValueError("No existe una pose inicial con clearance suficiente")
        obstacles = tuple(
            (
                (float(column) + 0.5) * target_resolution - origin_x,
                origin_y - (float(row) + 0.5) * target_resolution,
            )
            for row, column in zip(occupied_rows, occupied_columns)
        )
        return SimulationMap(
            obstacles,
            robot_start_world=(0.0, 0.0),
            obstacle_size=target_resolution,
        )


__all__ = ["NpyOccupancyMapLoader"]
