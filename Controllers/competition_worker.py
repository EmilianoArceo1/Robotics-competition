"""Ejecución de competición fuera del hilo gráfico de Qt."""

from __future__ import annotations

from dataclasses import dataclass
import traceback

import numpy as np
from PySide6.QtCore import QObject, QThread, Signal, Slot

from Logic.Competition import CompetitionWorld


@dataclass(frozen=True, slots=True)
class CompetitionFrame:
    timestep: int
    max_steps: int
    coverage: float
    poses: tuple[tuple[int, int], ...]
    modes: tuple[str, ...]
    observed: np.ndarray


class CompetitionWorker(QObject):
    frame_ready = Signal(object)
    completed = Signal()
    failed = Signal(str)

    def __init__(self, world: CompetitionWorld, view_mode: str) -> None:
        super().__init__()
        self.world = world
        self.view_mode = view_mode
        self.paused = False
        self.stopped = False

    def _observed(self) -> np.ndarray:
        if self.view_mode == "Reportado a base":
            return self.world.base_obs_map.copy()
        if self.view_mode == "Robot 1":
            return self.world.robots[0].combined_obs_map.copy()
        return self.world.live_observation_map()

    @Slot()
    def run(self) -> None:
        try:
            while not self.stopped and self.world.timestep < self.world.config.max_steps:
                if self.paused:
                    QThread.msleep(20)
                    continue
                self.world.step()
                self.frame_ready.emit(CompetitionFrame(
                    self.world.timestep, self.world.config.max_steps,
                    self.world.coverage,
                    tuple(tuple(map(int, robot.pose)) for robot in self.world.robots),
                    tuple(robot.behavior_mode for robot in self.world.robots),
                    self._observed(),
                ))
            if not self.stopped:
                self.completed.emit()
        except Exception:
            self.failed.emit(traceback.format_exc())


__all__ = ["CompetitionFrame", "CompetitionWorker"]
