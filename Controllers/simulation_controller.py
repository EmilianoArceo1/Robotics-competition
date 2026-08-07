"""Control del ciclo de vida de una simulación."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from .belief_map_controller import BeliefMapController
from .map_controller import MapController
from .objective_assign_controller import ObjectiveAssignController
from .path_planner_controller import PathPlannerController
from .robot_control_controller import RobotControlController
from .safe_tracker_controller import SafeTrackerController
from Logic.Robot.CollisionChecker import CollisionChecker
from Logic.Robot.Track import Track
from .runtime import MotionRuntime, NavigationRuntime, PerceptionRuntime


@dataclass(frozen=True, slots=True)
class SimulationStatus:
    running: bool
    path_planner: str
    objective_assigner: str
    safe_tracker: str


class SimulationController:
    def __init__(
        self,
        map_controller: MapController,
        path_planner_controller: PathPlannerController,
        objective_assign_controller: ObjectiveAssignController,
        belief_map_controller: BeliefMapController | None = None,
        safe_tracker_controller: SafeTrackerController | None = None,
    ) -> None:
        self.map_controller = map_controller
        self.path_planner_controller = path_planner_controller
        self.objective_assign_controller = objective_assign_controller
        self.belief_map_controller = belief_map_controller
        self.safe_tracker_controller = safe_tracker_controller or SafeTrackerController()
        self.control: RobotControlController | None = None
        self.track: Track | None = None
        self._running = False
        self._sensor_fov = 360.0
        self._sensor_radius = 10.0
        self._grid_size = 1.0
        self._safety_radius = 0.20
        self.collision_checker: CollisionChecker | None = None
        self.perception_runtime = PerceptionRuntime(
            self.map_controller, self.belief_map_controller
        )
        self.navigation_runtime = NavigationRuntime(self.map_controller)
        self.motion_runtime = MotionRuntime(self.map_controller)

    @property
    def running(self) -> bool:
        return self._running

    @property
    def robot(self):
        return self.control.physics if self.control is not None else None

    @property
    def grid_size(self) -> float:
        return self._grid_size

    def start(self) -> SimulationStatus:
        """Inicializa una simulación y coloca el robot en su origen local."""
        self.map_controller.reset_robot()
        self.map_controller.update_robot_pose(0.0, 0.0, 0.0)
        objective_assigner = self.objective_assign_controller.create(
            self._grid_size
        )
        self.control = RobotControlController(
            objective_assigner,
            field_of_view=self._sensor_fov,
            detection_radius=self._sensor_radius,
            grid_size=self._grid_size,
            safe_tracker=self.safe_tracker_controller.create(),
        )
        if self.belief_map_controller is not None:
            self.belief_map_controller.configure_grid_size(self._grid_size)
            self.belief_map_controller.clear()
        self.map_controller.configure_sensor(
            self.control.sensor.detection_radius,
            self.control.sensor.field_of_view,
        )
        local_obstacles = tuple(
            (float(cell[0][0]), float(cell[0][1]))
            for cell in self.map_controller.simulation_map.sensor_matrix
        )
        geometry = self.control.physics.geometry
        physical_radius = (
            (geometry.length / 2.0) ** 2
            + (geometry.width / 2.0) ** 2
        ) ** 0.5
        self.collision_checker = CollisionChecker(
            local_obstacles,
            robot_radius=physical_radius,
            safety_radius=self._safety_radius,
        )
        self.track = self.path_planner_controller.create(
            self.control.physics,
            local_obstacles,
            self._grid_size,
        )
        self._running = True
        return self.status()

    def set_sensor_fov(self, degrees: float) -> None:
        value = float(degrees)
        if not 0.0 < value <= 360.0:
            raise ValueError("El FOV debe estar entre 0 y 360 grados")
        self._sensor_fov = value
        if self.control is not None:
            self.control.sensor.configure_field_of_view(value)
            self.map_controller.configure_sensor(
                self.control.sensor.detection_radius,
                value,
            )

    def set_sensor_radius(self, radius: float) -> None:
        value = float(radius)
        if not isfinite(value) or not 0.5 <= value <= 30.0:
            raise ValueError("La distancia del sensor debe estar entre 0.5 y 30 m")
        self._sensor_radius = value
        if self.control is not None:
            self.control.sensor.detection_radius = value
            self.map_controller.configure_sensor(
                value,
                self.control.sensor.field_of_view,
            )

    def set_safety_radius(self, radius: float) -> None:
        value = float(radius)
        if not isfinite(value) or not 0.0 <= value <= 5.0:
            raise ValueError("El radio de seguridad debe estar entre 0 y 5 m")
        self._safety_radius = value
        if self.collision_checker is not None:
            self.collision_checker.configure_safety_radius(value)

    def set_grid_size(self, grid_size: float) -> SimulationStatus:
        """Configura la resolución y reinicia para no mezclar cuadrículas."""
        value = float(grid_size)
        if not isfinite(value) or not 0.1 <= value <= 5.0:
            raise ValueError("El tamaño del grid debe estar entre 0.1 y 5.0")
        changed = value != self._grid_size
        self._grid_size = value
        if self.belief_map_controller is not None:
            self.belief_map_controller.configure_grid_size(value)
        if changed and self._running:
            return self.start()
        return self.status()

    def step(self, dt: float) -> None:
        """Ejecuta un ciclo completo de percepción, decisión y movimiento."""
        if not self._running or self.control is None or self.track is None:
            return
        if dt <= 0.0:
            raise ValueError("dt debe ser mayor que cero")

        self.perception_runtime.run(self.control, self._grid_size)
        self.navigation_runtime.ensure_route(self.control, self.track)
        self.motion_runtime.run(
            self.control,
            self.track,
            self.collision_checker,
            dt,
        )
        self.navigation_runtime.publish_route(self.track)

    def stop(self) -> SimulationStatus:
        if self.robot is not None:
            self.robot.stop()
        self._running = False
        return self.status()

    def move_robot(self, x: float, y: float) -> bool:
        """Actualiza simultáneamente la vista del mapa y las físicas."""
        x, y = float(x), float(y)
        if self.collision_checker is not None and self.collision_checker.collides((x, y)):
            return False
        if self.robot is not None:
            self.robot.state.x = x
            self.robot.state.y = y
            self.robot.stop()
            theta = self.robot.state.theta
        else:
            theta = 0.0
        self.map_controller.update_robot_pose(x, y, theta)
        if self.track is not None:
            self.track.waypoints.replace(())
        return True

    def status(self) -> SimulationStatus:
        return SimulationStatus(
            running=self._running,
            path_planner=self.path_planner_controller.selected_method,
            objective_assigner=(
                self.objective_assign_controller.selected_method
            ),
            safe_tracker=self.safe_tracker_controller.selected_method,
        )
