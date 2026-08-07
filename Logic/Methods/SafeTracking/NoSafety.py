"""Filtro neutro usado cuando no se ha seleccionado una estrategia segura."""

from __future__ import annotations

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotState
from Logic.Robot.Sensor import SensorScan
from .SafeTracker import ControlCommand, SafeTracker


class NoSafety(SafeTracker):
    def filter_control(
        self,
        robot_state: RobotState,
        nominal_control: ControlCommand,
        belief_map: BeliefMap,
        sensor_scan: SensorScan | None,
        dt: float,
    ) -> ControlCommand:
        if dt <= 0.0:
            raise ValueError("dt debe ser mayor que cero")
        return nominal_control
