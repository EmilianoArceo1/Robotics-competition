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
from .clustering_controller import ClusteringController
from .experiment_controller import ExperimentController
from .coordination_controller import CoordinationController
from Logic.Methods.Coordination import CoordinationTransport
from Logic.Robot.CollisionChecker import CollisionChecker, CollisionReport
from Logic.Robot.Track import Track
from Logic.Robot.Fleet import RobotFleet
from Logic.Exploration import ExplorationMetrics, ExplorationOutcome
from Logic.Experiments import (
    AlgorithmConfiguration,
    ExperimentConfiguration,
    MapConfiguration,
    RobotConfiguration,
    SensorConfiguration,
)
from .runtime import (
    CoordinationRuntime, MotionRuntime, NavigationRuntime, PerceptionRuntime,
)


@dataclass(frozen=True, slots=True)
class SimulationStatus:
    running: bool
    path_planner: str
    objective_assigner: str
    safe_tracker: str
    coordination: str


class SimulationController:
    def __init__(
        self,
        map_controller: MapController,
        path_planner_controller: PathPlannerController,
        objective_assign_controller: ObjectiveAssignController,
        belief_map_controller: BeliefMapController | None = None,
        safe_tracker_controller: SafeTrackerController | None = None,
        clustering_controller: ClusteringController | None = None,
        experiment_controller: ExperimentController | None = None,
        coordination_controller: CoordinationController | None = None,
        coordination_transport: CoordinationTransport | None = None,
    ) -> None:
        self.map_controller = map_controller
        self.path_planner_controller = path_planner_controller
        self.objective_assign_controller = objective_assign_controller
        self.belief_map_controller = belief_map_controller
        self.safe_tracker_controller = safe_tracker_controller or SafeTrackerController()
        self.clustering_controller = clustering_controller or ClusteringController()
        self.experiment_controller = experiment_controller or ExperimentController()
        self.coordination_controller = coordination_controller or CoordinationController()
        self.coordination_transport = coordination_transport
        self.control: RobotControlController | None = None
        self.track: Track | None = None
        self._running = False
        self._sensor_fov = 360.0
        self._sensor_radius = 10.0
        self._grid_size = 1.0
        self._safety_radius = 0.20
        self._robot_count = 1
        self._initial_robot_poses: tuple[tuple[float, float, float], ...] | None = None
        self.fleet: RobotFleet | None = None
        self._experiment_name = "exploration-run"
        self._experiment_seed = 0
        self._simulation_time = 0.0
        self.collision_checker: CollisionChecker | None = None
        self.last_collision_report = CollisionReport(False)
        self.perception_runtime = PerceptionRuntime(
            self.map_controller, self.belief_map_controller
        )
        self.navigation_runtime = NavigationRuntime(self.map_controller)
        self.coordination_runtime = CoordinationRuntime(
            self.coordination_controller.create(),
            transport=self.coordination_transport,
        )
        self.motion_runtime = MotionRuntime(self.map_controller)
        self.exploration_metrics = ExplorationMetrics()

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
        if self.experiment_controller.manager.active:
            self.experiment_controller.finish(
                self.metrics_snapshot,
                outcome=ExplorationOutcome.ABORTED.value,
                reason="replaced by a new experiment",
            )
        self.map_controller.reset_robot()
        self.map_controller.update_robot_pose(0.0, 0.0, 0.0)
        objective_assigner = self.objective_assign_controller.create(
            self._grid_size,
            self.clustering_controller.create(),
        )
        self.control = RobotControlController(
            objective_assigner,
            field_of_view=self._sensor_fov,
            detection_radius=self._sensor_radius,
            grid_size=self._grid_size,
            safe_tracker=self.safe_tracker_controller.create(self._safety_radius),
        )
        self.fleet = RobotFleet(self.control.physics, self._robot_count)
        if self._initial_robot_poses:
            start_x, start_y = self.map_controller.simulation_map.robot_start_world
            for member, (x, y, theta) in zip(self.fleet.members, self._initial_robot_poses):
                member.physics.state.x = float(x) - start_x
                member.physics.state.y = float(y) - start_y
                member.physics.state.theta = float(theta)
        self.map_controller.update_fleet(self.fleet.members)
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
            obstacle_size=self.map_controller.simulation_map.obstacle_size,
        )
        self.last_collision_report = CollisionReport(False)
        self.navigation_runtime.reset()
        self._simulation_time = 0.0
        clear_transport = getattr(self.coordination_transport, "clear", None)
        if callable(clear_transport):
            clear_transport()
        self.coordination_runtime = CoordinationRuntime(
            self.coordination_controller.create(),
            transport=self.coordination_transport,
        )
        self.exploration_metrics.reset()
        self.track = self.path_planner_controller.create(
            self.control.physics,
            local_obstacles,
            self._grid_size,
            self._safety_radius,
        )
        map_model = self.map_controller.simulation_map
        robot_geometry = self.control.physics.geometry
        self.experiment_controller.start(
            ExperimentConfiguration(
                self._experiment_name,
                self._experiment_seed,
                AlgorithmConfiguration(
                    self.path_planner_controller.selected_method,
                    self.objective_assign_controller.selected_method,
                    self.clustering_controller.selected_method,
                    self.safe_tracker_controller.selected_method,
                    self.coordination_controller.selected_method,
                ),
                SensorConfiguration(
                    self.control.sensor.sensor_type,
                    self._sensor_fov,
                    self._sensor_radius,
                    self._grid_size,
                ),
                RobotConfiguration(
                    robot_geometry.length,
                    robot_geometry.width,
                    self._safety_radius,
                    self._robot_count,
                ),
                MapConfiguration(
                    map_model.obstacles,
                    map_model.obstacle_size,
                    map_model.robot_start_world,
                ),
            )
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
        if self.control is not None:
            configure = getattr(
                self.control.safe_tracker, "configure_safety_radius", None
            )
            if callable(configure):
                configure(value)
        if self.track is not None:
            configure_planner = getattr(
                self.track, "configure_safety_radius", None
            )
            if callable(configure_planner):
                configure_planner(value)

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

    def set_robot_count(self, count: int) -> SimulationStatus:
        value = int(count)
        if not 1 <= value <= 20:
            raise ValueError("El número de robots debe estar entre 1 y 20")
        self._robot_count = value
        self._initial_robot_poses = None
        if self._running:
            return self.start()
        self.map_controller.configure_robot_preview(value)
        return self.status()

    def set_initial_robot_poses(self, poses) -> None:
        values = tuple((float(x), float(y), float(theta)) for x, y, theta in poses)
        if not values or len(values) > 20:
            raise ValueError("El escenario debe contener entre 1 y 20 robots")
        if not all(isfinite(value) for pose in values for value in pose):
            raise ValueError("Las poses iniciales deben contener números finitos")
        self._initial_robot_poses = values
        self._robot_count = len(values)
        self.map_controller.configure_robot_poses(values)

    def step(self, dt: float) -> None:
        """Ejecuta un ciclo completo de percepción, decisión y movimiento."""
        if not self._running or self.control is None or self.track is None:
            return
        if dt <= 0.0:
            raise ValueError("dt debe ser mayor que cero")

        perception = self.perception_runtime.run(self.control, self._grid_size)
        self._simulation_time += dt
        coordination = self.coordination_runtime.run(
            self.control,
            timestamp=self._simulation_time,
            navigation_state=self.navigation_snapshot.state.value,
            current_goal=self.navigation_snapshot.current_goal,
            fleet_members=(self.fleet.members if self.fleet is not None else None),
        )
        self.navigation_runtime.ensure_route(
            self.control,
            self.track,
            coordination.ordered_goals,
            coordination_error=coordination.error,
        )
        self.last_collision_report = self.motion_runtime.run(
            self.control,
            self.track,
            self.collision_checker,
            dt,
        )
        self.navigation_runtime.observe_motion(
            self.control,
            self.track,
            collision=self.last_collision_report.collision,
        )
        self.navigation_runtime.publish_route(self.track)
        if self.fleet is not None:
            self.map_controller.update_fleet(self.fleet.members)
        tracker_status = getattr(self.control.safe_tracker, "status", None)
        self.exploration_metrics.record_tick(
            dt=dt,
            position=(self.control.physics.state.x, self.control.physics.state.y),
            belief_matrix=self.control.belief_map.matrix,
            total_cells=perception.total_cells,
            exploration=self.exploration_snapshot,
            navigation=self.navigation_snapshot,
            safety_active=bool(
                tracker_status is not None and tracker_status.active
            ),
            collision_rejected=self.last_collision_report.collision,
        )
        self.experiment_controller.record(
            metrics=self.metrics_snapshot,
            pose=(
                self.control.physics.state.x,
                self.control.physics.state.y,
                self.control.physics.state.theta,
            ),
            navigation=self.navigation_snapshot,
            exploration=self.exploration_snapshot,
            safety_active=bool(
                tracker_status is not None and tracker_status.active
            ),
            collision_rejected=self.last_collision_report.collision,
        )
        if self.metrics_snapshot.outcome != ExplorationOutcome.RUNNING:
            self.experiment_controller.finish(self.metrics_snapshot)
            self.control.physics.stop()
            self._running = False

    def stop(self) -> SimulationStatus:
        if self.robot is not None:
            self.robot.stop()
        if self.experiment_controller.manager.active:
            self.experiment_controller.finish(
                self.metrics_snapshot,
                outcome=ExplorationOutcome.ABORTED.value,
                reason="simulation stopped by user",
            )
        self._running = False
        return self.status()

    def reset(self) -> SimulationStatus:
        """Detiene y limpia el escenario sin crear otro experimento."""
        if self._running:
            self.stop()
        self.map_controller.reset_robot()
        self.map_controller.update_robot_pose(0.0, 0.0, 0.0)
        if self.robot is not None:
            state = self.robot.state
            state.x = state.y = state.theta = 0.0
            self.robot.stop()
        if self.track is not None:
            self.track.waypoints.replace(())
        if self.control is not None:
            self.control.belief_map.clear()
        if self.belief_map_controller is not None:
            self.belief_map_controller.clear()
        self.navigation_runtime.reset()
        self.exploration_metrics.reset()
        self.map_controller.configure_robot_preview(self._robot_count)
        return self.status()

    def move_robot(self, x: float, y: float) -> bool:
        """Actualiza simultáneamente la vista del mapa y las físicas."""
        x, y = float(x), float(y)
        if self.collision_checker is not None:
            report = self.collision_checker.check_position((x, y))
            self.last_collision_report = report
            if report.collision:
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
            self.navigation_runtime.supervisor.route_invalidated(
                "robot moved manually"
            )
        return True

    @property
    def navigation_snapshot(self):
        return self.navigation_runtime.supervisor.snapshot

    @property
    def exploration_snapshot(self):
        return self.navigation_runtime.exploration.snapshot

    @property
    def metrics_snapshot(self):
        return self.exploration_metrics.snapshot

    @property
    def experiment_result(self):
        return self.experiment_controller.result

    def configure_experiment(self, *, name: str, seed: int) -> None:
        value = name.strip()
        if not value:
            raise ValueError("El nombre del experimento es obligatorio")
        self._experiment_name = value
        self._experiment_seed = int(seed)

    def save_experiment_json(self, destination: str) -> None:
        self.experiment_controller.save_json(destination)

    def export_experiment_csv(self, destination: str) -> None:
        self.experiment_controller.export_csv(destination)

    def status(self) -> SimulationStatus:
        return SimulationStatus(
            running=self._running,
            path_planner=self.path_planner_controller.selected_method,
            objective_assigner=(
                self.objective_assign_controller.selected_method
            ),
            safe_tracker=self.safe_tracker_controller.selected_method,
            coordination=self.coordination_controller.selected_method,
        )
