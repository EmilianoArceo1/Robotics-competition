"""Interfaz para filtros de seguimiento seguro."""

from __future__ import annotations

from abc import ABC, abstractmethod

from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotState
from Logic.Robot.Sensor import SensorScan

ControlCommand = tuple[float, float]


class SafeTracker(ABC):
    """Última barrera entre el seguimiento nominal y las físicas."""

    @abstractmethod
    def filter_control(
        self,
        robot_state: RobotState,
        nominal_control: ControlCommand,
        belief_map: BeliefMap,
        sensor_scan: SensorScan | None,
        dt: float,
    ) -> ControlCommand:
        """Devuelve aceleraciones lineal y angular seguras."""
        raise NotImplementedError
