"""Composición principal de los componentes del robot."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Iterable, Sequence
from math import isfinite

from Logic.Methods.SafeTracking import NoSafety, SafeTracker
from .BeliefMap import BeliefMap
from .Physic import ControlLimits, RobotGeometry, RobotPhysics, RobotState
from .Sensor import Sensor, SensorMatrix, SensorScan
from .Track import Track
from .waypoints import Waypoints


class Control(ABC):
    def __init__(
        self,
        *,
        geometry: RobotGeometry | None = None,
        limits: ControlLimits | None = None,
        initial_state: RobotState | None = None,
        sensor_type: str = "lidar",
        detection_radius: float = 10.0,
        field_of_view: float = 360.0,
        grid_size: float = 1.0,
        safe_tracker: SafeTracker | None = None,
    ) -> None:
        self.physics = RobotPhysics(
            geometry=geometry or RobotGeometry(),
            limits=limits or ControlLimits(),
            state=initial_state or RobotState(),
        )
        self.sensor = Sensor(
            sensor_type=sensor_type,
            detection_radius=detection_radius,
            field_of_view=field_of_view,
            grid_size=grid_size,
        )
        self.belief_map = BeliefMap()
        self.last_scan: SensorScan | None = None
        self.safe_tracker = safe_tracker or NoSafety()

    def detect(
        self, environment_matrix: Iterable[Sequence[object]]
    ) -> SensorMatrix:
        self.last_scan = self.sensor.scan(
            self.physics.state, environment_matrix
        )
        observations = self.last_scan.detected
        self.belief_map.update(observations)
        return observations

    @abstractmethod
    def assign_goal(
        self,
        belief_map: BeliefMap,
        robot_state: RobotState,
    ) -> list[float]:
        """Selecciona y devuelve una única meta [x, y]."""
        raise NotImplementedError

    def create_route_to_assigned_goal(self, track: Track) -> Waypoints:
        if not isinstance(track, Track):
            raise TypeError("track debe ser una instancia concreta de Track")
        if track.robot is not self.physics:
            raise ValueError(
                "track y control deben compartir la misma instancia de RobotPhysics"
            )
        state = self.physics.state
        goal = self.assign_goal(self.belief_map, state)
        if not isinstance(goal, Sequence) or isinstance(goal, (str, bytes)):
            raise TypeError("assign_goal debe devolver una coordenada [x, y]")
        if len(goal) != 2:
            raise ValueError("assign_goal debe devolver exactamente [x, y]")
        goal_x, goal_y = float(goal[0]), float(goal[1])
        if not isfinite(goal_x) or not isfinite(goal_y):
            raise ValueError("La meta debe contener coordenadas finitas")
        return track.create_route(
            [state.x, state.y], [goal_x, goal_y]
        )

    def apply_tracking_control(self, track: Track, dt: float) -> bool:
        """Filtra el control nominal y sólo entonces lo entrega a las físicas.

        Devuelve ``True`` cuando la ruta ha terminado.
        """
        if not isinstance(track, Track):
            raise TypeError("track debe ser una instancia concreta de Track")
        if track.robot is not self.physics:
            raise ValueError(
                "track y control deben compartir la misma instancia de RobotPhysics"
            )
        if dt <= 0.0:
            raise ValueError("dt debe ser mayor que cero")

        nominal_control = track.compute_control()
        if nominal_control is None:
            self.physics.stop()
            return True

        safe_control = self.safe_tracker.filter_control(
            robot_state=self.physics.state,
            nominal_control=nominal_control,
            belief_map=self.belief_map,
            sensor_scan=self.last_scan,
            dt=dt,
        )
        if len(safe_control) != 2:
            raise ValueError("SafeTracker debe devolver (aceleración lineal, angular)")
        linear_acceleration = float(safe_control[0])
        angular_acceleration = float(safe_control[1])
        if not isfinite(linear_acceleration) or not isfinite(angular_acceleration):
            raise ValueError("SafeTracker debe devolver aceleraciones finitas")
        self.physics.apply_control(
            linear_acceleration,
            angular_acceleration,
        )
        return False
