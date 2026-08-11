"""Carga segura del formato JSON ``robotics_sim_lab.sim``."""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import ceil, floor, isfinite
from pathlib import Path
from typing import Any

from Logic.Map.maps import SimulationMap


@dataclass(frozen=True, slots=True)
class SimFile:
    simulation_map: SimulationMap
    world_bounds: tuple[float, float, float, float]
    robot_poses: tuple[tuple[float, float, float], ...]
    grid_resolution: float
    sensor_range: float
    sensor_fov: float
    safety_radius: float
    competition_occ_map: Path | None = None
    competition_valid_space: Path | None = None
    competition_config: dict[str, Any] | None = None


class SimFileLoader:
    SCHEMA = "robotics_sim_lab.sim"
    VERSION = 1

    def load(self, source: str | Path) -> SimFile:
        path = Path(source)
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise ValueError(f"El archivo .sim no contiene JSON válido: línea {error.lineno}") from error
        if not isinstance(payload, dict):
            raise ValueError("La raíz del archivo .sim debe ser un objeto")
        if payload.get("schema") != self.SCHEMA:
            raise ValueError(f"Esquema .sim no compatible: {payload.get('schema')!r}")
        if payload.get("version") != self.VERSION:
            raise ValueError(f"Versión .sim no compatible: {payload.get('version')!r}")

        world = self._object(payload, "world")
        bounds = (
            self._number(world, "x_min"), self._number(world, "y_min"),
            self._number(world, "x_max"), self._number(world, "y_max"),
        )
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError("Los límites del mundo .sim no son válidos")

        map_data = self._object(payload, "map")
        resolution = self._number(map_data, "grid_resolution")
        if not 0.01 <= resolution <= 5.0:
            raise ValueError("map.grid_resolution debe estar entre 0.01 y 5 m")
        rectangles = map_data.get("obstacles", [])
        if not isinstance(rectangles, list):
            raise ValueError("map.obstacles debe ser una lista")
        competition = payload.get("competition")
        competition_occ_map = None
        competition_valid_space = None
        competition_config = None
        if competition is not None:
            if not isinstance(competition, dict):
                raise ValueError("competition debe ser un objeto")
            occ_value = competition.get("occ_map")
            if not isinstance(occ_value, str) or not occ_value:
                raise ValueError("competition.occ_map debe indicar un archivo")
            competition_occ_map = (path.parent / occ_value).resolve()
            if not competition_occ_map.is_file():
                raise ValueError(f"No existe el mapa de competición: {competition_occ_map}")
            valid_value = competition.get("valid_space")
            if isinstance(valid_value, str):
                competition_valid_space = (path.parent / valid_value).resolve()
                if not competition_valid_space.is_file():
                    raise ValueError(f"No existe valid_space: {competition_valid_space}")
            config_value = competition.get("config", {})
            if not isinstance(config_value, dict):
                raise ValueError("competition.config debe ser un objeto")
            competition_config = dict(config_value)
            obstacles = self._competition_obstacles(competition_occ_map, resolution)
        else:
            obstacles = self._rasterize(rectangles, resolution)

        robot = self._object(payload, "robot")
        fallback_pose = self._pose(robot, "robot")
        multi = payload.get("multi_robot", {})
        robots = multi.get("robots", []) if isinstance(multi, dict) else []
        poses = tuple(self._pose(value, f"multi_robot.robots[{index}]") for index, value in enumerate(robots))
        if not poses:
            poses = (fallback_pose,)
        start = poses[0][:2]

        sensor = payload.get("sensor", {})
        if not isinstance(sensor, dict):
            sensor = {}
        sensor_range = self._optional_number(sensor, "range", 10.0)
        sensor_fov = self._optional_number(sensor, "camera_fov_degrees", 360.0)
        safety_radius = self._optional_number(robot, "safety_radius", 0.20)
        # El controlador maneja radio de seguridad adicional al cuerpo.
        body_radius = self._optional_number(robot, "body_radius", 0.0)
        safety_margin = max(0.0, safety_radius - body_radius)

        return SimFile(
            SimulationMap(obstacles, robot_start_world=start,
                          obstacle_size=resolution, world_bounds=bounds),
            bounds, poses, resolution, sensor_range, sensor_fov, safety_margin,
            competition_occ_map, competition_valid_space, competition_config,
        )

    @staticmethod
    def _competition_obstacles(source: Path, resolution: float) -> tuple[tuple[float, float], ...]:
        import numpy as np
        raw = np.load(source, allow_pickle=False)
        if raw.ndim != 2 or not set(int(v) for v in np.unique(raw)).issubset({0, 254, 255}):
            raise ValueError("El mapa de competición debe ser 2D con valores 0/254/255")
        # La competición reduce bloques 2x2. El visualizador usa la misma
        # topología, pero permite una resolución métrica declarada en el .sim.
        rows, columns = (raw.shape[0] + 1) // 2, (raw.shape[1] + 1) // 2
        padded = np.pad(raw, ((0, rows*2-raw.shape[0]), (0, columns*2-raw.shape[1])), constant_values=0)
        reduced = padded.reshape(rows, 2, columns, 2).min(axis=(1, 3))
        occupied_rows, occupied_columns = np.where(reduced == 0)
        return tuple((float(column) * resolution, -float(row) * resolution)
                     for row, column in zip(occupied_rows, occupied_columns))

    @staticmethod
    def _object(payload: dict[str, Any], key: str) -> dict[str, Any]:
        value = payload.get(key)
        if not isinstance(value, dict):
            raise ValueError(f"{key} debe ser un objeto")
        return value

    @staticmethod
    def _number(payload: dict[str, Any], key: str) -> float:
        value = payload.get(key)
        if isinstance(value, bool):
            raise ValueError(f"{key} debe ser numérico")
        try:
            result = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{key} debe ser numérico") from error
        if not isfinite(result):
            raise ValueError(f"{key} debe ser finito")
        return result

    def _optional_number(self, payload: dict[str, Any], key: str, default: float) -> float:
        return default if key not in payload else self._number(payload, key)

    def _pose(self, payload: Any, name: str) -> tuple[float, float, float]:
        if not isinstance(payload, dict):
            raise ValueError(f"{name} debe ser un objeto")
        return self._number(payload, "x"), self._number(payload, "y"), self._optional_number(payload, "theta", 0.0)

    def _rasterize(self, rectangles: list[Any], resolution: float) -> tuple[tuple[float, float], ...]:
        cells: set[tuple[int, int]] = set()
        for index, rectangle in enumerate(rectangles):
            if not isinstance(rectangle, list) or len(rectangle) != 4:
                raise ValueError(f"map.obstacles[{index}] debe ser [x, y, ancho, alto]")
            try:
                x, y, width, height = (float(value) for value in rectangle)
            except (TypeError, ValueError) as error:
                raise ValueError(f"map.obstacles[{index}] contiene valores no numéricos") from error
            if not all(isfinite(value) for value in (x, y, width, height)) or width <= 0 or height <= 0:
                raise ValueError(f"map.obstacles[{index}] debe tener dimensiones positivas y finitas")
            first_x, last_x = floor(x / resolution), ceil((x + width) / resolution) - 1
            first_y, last_y = floor(y / resolution), ceil((y + height) / resolution) - 1
            cells.update((column, row) for row in range(first_y, last_y + 1) for column in range(first_x, last_x + 1))
        return tuple(sorted(((column + 0.5) * resolution, (row + 0.5) * resolution) for column, row in cells))
