"""Ventana principal profesional basada en Qt 6."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QSettings, Qt, QThread, QTimer
from PySide6.QtWidgets import (
    QDockWidget, QFileDialog, QMainWindow, QMenu, QScrollArea, QWidget,
)

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.simulation_controller import SimulationController
from Controllers.safe_tracker_controller import SafeTrackerController
from Controllers.clustering_controller import ClusteringController
from Controllers.experiment_controller import ExperimentController
from Controllers.coordination_controller import CoordinationController
from Controllers.competition_worker import CompetitionFrame, CompetitionWorker
from Infrastructure.Persistence import CsvExperimentExporter, JsonExperimentRepository
from Infrastructure.Communication import InMemoryCoordinationTransport
from Infrastructure.Maps import NpyOccupancyMapLoader
from Infrastructure.Simulation import SimFileLoader
from Logic.Competition import (CompetitionConfig, CompetitionWorld, UtilityWeights,
                               WeightedUtilityPolicy, NextBestViewPolicy,
                               create_information_gain, make_advanced_policy)
from Logic.Competition.return_policies import make_return_policy
from Logic.Competition.handoff_policies import make_handoff_policy
from Logic.Competition.environment import load_policy
from Logic.Competition.experiments import (ENV_SIZE, TrialResult, export_csv,
                                           leave_one_environment_out, summarize)
from .configuration_panel import ConfigurationPanel
from .map_view import MapView
from .title_bar import TitleBar
from .visual_settings import VisualSettingsDialog


STYLE = """
QMainWindow, QWidget { background: #0b1220; color: #dbeafe; font: 10pt 'Segoe UI'; }
#sidePanel { background: #101b2d; border-right: 1px solid #22324a; }
#brand { color: #38bdf8; font-size: 18px; font-weight: 800; letter-spacing: 2px; }
#muted { color: #718096; font-size: 11px; }
QGroupBox { color: #7dd3fc; font-size: 10px; font-weight: 700; border: 1px solid #263750;
  border-radius: 10px; margin-top: 12px; padding: 14px 10px 10px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit { background: #17243a; border: 1px solid #334765; border-radius: 7px;
  padding: 7px; color: #f1f5f9; min-height: 22px; }
QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover, QLineEdit:hover { border-color: #38bdf8; }
QComboBox QAbstractItemView { background: #17243a; color: #f1f5f9; selection-background-color: #0369a1; }
QCheckBox { spacing: 9px; padding: 4px; }
QCheckBox::indicator { width: 17px; height: 17px; }
QPushButton { background: #1e293b; border: 1px solid #3b4d68; border-radius: 8px; padding: 8px; }
QPushButton:hover { border-color: #38bdf8; background: #24344e; }
#primaryButton { background: #0284c7; border: none; color: white; font-weight: 800; }
#primaryButton:hover { background: #0ea5e9; }
#statusPill { color: #94a3b8; background: #172033; border-radius: 8px; padding: 8px; font-size: 9px; }
#statusPill[running="true"] { color: #86efac; background: #123126; }
#mapCanvas { border: 1px solid #22324a; border-radius: 12px; }
#titleBar { background: #111c2e; border-bottom: 1px solid #263750; }
#windowMark { color: #38bdf8; font-size: 20px; font-weight: 800; }
#windowTitle { color: #cbd5e1; font-size: 10px; font-weight: 700; letter-spacing: 1px; }
#windowButton, #closeButton { background: transparent; border: none; border-radius: 5px;
  min-width: 38px; max-width: 38px; min-height: 30px; padding: 0; font-size: 16px; }
#windowButton:hover { background: #263750; }
#closeButton:hover { background: #dc2626; color: white; }
#secondaryButton { background: transparent; border: 1px solid #3b82f6; color: #93c5fd; font-weight: 700; }
QScrollArea { border: none; background: #101b2d; }
QScrollBar:vertical { background: #101b2d; width: 8px; margin: 2px; }
QScrollBar::handle:vertical { background: #334765; border-radius: 4px; min-height: 30px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""


class MainView(QMainWindow):
    def __init__(self, map_controller: MapController | None = None) -> None:
        super().__init__()
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.Window)
        self.setWindowTitle("Exploration Lab — Robot Simulator")
        self.resize(1280, 820)
        self.setMinimumSize(900, 600)
        self.setStyleSheet(STYLE)
        self.settings = QSettings("ExplorationLab", "RobotSimulator")
        if map_controller is None:
            competition_map = (
                Path(__file__).resolve().parents[1]
                / "Assets" / "maps" / "env3" / "occ_map.npy"
            )
            simulation_map = NpyOccupancyMapLoader().load(
                str(competition_map),
                start_pose=(15.0, 15.0),
                source_resolution=0.05,
                target_resolution=0.5,
            )
            self.map_controller = MapController(simulation_map)
        else:
            self.map_controller = map_controller
        self.belief_controller = BeliefMapController(self.map_controller.simulation_map)
        for occupancy in (-1, 0, 1):
            saved = self.settings.value(f"visual/belief_{occupancy}")
            if saved:
                self.belief_controller.set_color(occupancy, str(saved))
        self.path_controller = PathPlannerController()
        self.objective_controller = ObjectiveAssignController()
        self.safe_tracker_controller = SafeTrackerController()
        self.clustering_controller = ClusteringController()
        self.coordination_controller = CoordinationController()
        self.coordination_transport = InMemoryCoordinationTransport()
        self.experiment_controller = ExperimentController(
            JsonExperimentRepository(), CsvExperimentExporter()
        )
        self.simulation = SimulationController(
            self.map_controller, self.path_controller, self.objective_controller,
            self.belief_controller, self.safe_tracker_controller,
            self.clustering_controller,
            self.experiment_controller,
            self.coordination_controller,
            self.coordination_transport,
        )
        self.map_view = MapView(
            self.map_controller, self.belief_controller, self._move_robot
        )
        self.map_view.set_robot_style(str(self.settings.value("visual/robot", "Círculo")))
        self.panel = ConfigurationPanel(
            self.path_controller, self.objective_controller, self.belief_controller,
            self.safe_tracker_controller,
            self.clustering_controller,
            self.coordination_controller,
            self._start, self._toggle_frontiers, self._toggle_route,
            self._toggle_obstacles, self._toggle_clusters,
            self._change_fov, self._change_sensor_radius,
            self._change_safety_radius,
            self._change_grid, self._visual_changed,
            self._toggle_pause,
            self._export_experiment_json,
            self._export_experiment_csv,
            self._change_robot_count,
            self._load_sim_file,
            self._load_policy_file,
            self._change_policy_weights,
            self._export_competition_table,
            self.map_view.set_robot_visual_scale,
            self._change_competition_map_view,
            self._change_information_gain,
            self._change_competition_policy,
            self._change_return_policy,
            self._change_handoff_policy,
        )
        self.panel.experiment_name.setText(
            str(self.settings.value("experiment/name", "exploration-run"))
        )
        self.panel.experiment_seed.setValue(
            int(self.settings.value("experiment/seed", 0))
        )
        panel_scroll = QScrollArea()
        panel_scroll.setWidgetResizable(True)
        panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        panel_scroll.setWidget(self.panel)
        self.setMenuWidget(TitleBar(self, self._open_visual_settings))
        self.setCentralWidget(self.map_view)
        controls_dock = self._dock("Configuración", panel_scroll, "controlsDock")
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, controls_dock)
        self.setDockOptions(
            QMainWindow.DockOption.AllowNestedDocks
            | QMainWindow.DockOption.AllowTabbedDocks
            | QMainWindow.DockOption.AnimatedDocks
        )
        saved_layout = self.settings.value("workspace/layout")
        if saved_layout:
            self.restoreState(saved_layout)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self._step)
        self._paused = False
        self._competition_source: Path | None = None
        self._competition_config: CompetitionConfig | None = None
        self._competition_world: CompetitionWorld | None = None
        self._competition_policy_path: Path | None = None
        self._utility_weights = UtilityWeights()
        self._competition_trials: list[TrialResult] = []
        self._competition_thread: QThread | None = None
        self._competition_worker: CompetitionWorker | None = None
        self._last_competition_frame: CompetitionFrame | None = None
        self._competition_map_view = "Exploración en vivo"
        self._information_gain_name = "Unknown cells (circular)"
        self._information_gain_radius = 25
        self._competition_policy_name = "Weighted Frontier"
        self._return_policy_name = "periodic"
        self._handoff_policy_name = "closest_progress"

    def _change_handoff_policy(self, name: str) -> None:
        self._handoff_policy_name = {
            "Closest Progress": "closest_progress",
            "Payload Progress": "payload_progress",
            "Time Saving": "time_saving",
            "Returning Courier": "returning_courier",
            "Link Quality": "link_quality",
        }[name]

    def _change_return_policy(self, name: str) -> None:
        self._return_policy_name = {
            "Periodic": "periodic",
            "Deadline": "deadline",
            "Payload Adaptive": "payload_adaptive",
            "Link Aware": "link_aware",
            "Just In Time": "just_in_time",
            "Efficient Periodic": "efficient_periodic",
            "Selective Courier": "selective_courier",
            "Value Density": "value_density",
            "Nearest Frontier Return": "nearest_frontier_return",
            "Gain Sweep Return": "gain_sweep_return",
            "Homeward Sweep Return": "homeward_sweep_return",
        }[name]

    def _change_competition_policy(self, name: str) -> None:
        self._competition_policy_name = name
        if name != "External file":
            self._competition_policy_path = None
        if name in ("Next Best View", "Tuned NBV", "Adaptive NBV", "Gain per Cost",
                    "Coordinated Occlusion"):
            self.panel.information_gain_selector.setCurrentText(
                "Potential visibility (raycast)"
            )
            self.panel.information_gain_radius.setValue(100)

    def _change_competition_map_view(self, name: str) -> None:
        self._competition_map_view = name
        if self._competition_worker is not None:
            self._competition_worker.view_mode = name

    def _change_information_gain(self, name: str, radius: int) -> None:
        self._information_gain_name = name
        self._information_gain_radius = int(radius)

    def _change_policy_weights(self, information_gain: float, travel_cost: float,
                               redundancy: float, relay_risk: float) -> None:
        try:
            self._utility_weights = UtilityWeights(
                information_gain, travel_cost, redundancy, relay_risk
            )
        except ValueError:
            return

    def _export_competition_table(self) -> None:
        if not self._competition_trials:
            self.panel.show_error("Aún no hay ejecuciones de competición terminadas")
            return
        destination, _ = QFileDialog.getSaveFileName(
            self, "Exportar tabla experimental", "summary.csv", "CSV (*.csv)"
        )
        if not destination:
            return
        path = Path(destination)
        export_csv(summarize(self._competition_trials), path)
        export_csv(self._competition_trials, path.with_name(f"{path.stem}_raw.csv"))
        folds = leave_one_environment_out(self._competition_trials)
        if folds:
            export_csv(folds, path.with_name(f"{path.stem}_leave_one_out.csv"))
        self.panel.status.setText(f"TABLA EXPORTADA · {path.name}")

    def _load_policy_file(self) -> None:
        source, _ = QFileDialog.getOpenFileName(
            self, "Cargar policy de competición", "", "Policy Python (*.py)"
        )
        if not source:
            return
        try:
            load_policy(source)  # validar antes de conservarla
            self._competition_policy_path = Path(source)
            self.panel.competition_policy_selector.setCurrentText("External file")
            self.panel.status.setText(f"POLICY CARGADA · {Path(source).name}")
        except (OSError, ValueError, TypeError, ImportError) as error:
            self.panel.show_error(str(error))

    def _load_sim_file(self) -> None:
        source, _ = QFileDialog.getOpenFileName(
            self, "Cargar escenario", "", "Escenarios de simulación (*.sim);;Todos los archivos (*)"
        )
        if not source:
            return
        try:
            self.timer.stop()
            self._stop_competition_worker()
            self._competition_world = None
            if self.simulation.running:
                self.simulation.stop()
            loaded = SimFileLoader().load(source)
            self.map_controller.replace_map(loaded.simulation_map)
            self.belief_controller.replace_map(loaded.simulation_map)
            self.panel.apply_sim_configuration(
                grid_size=loaded.grid_resolution,
                sensor_range=loaded.sensor_range,
                sensor_fov=loaded.sensor_fov,
                safety_radius=loaded.safety_radius,
                robot_count=len(loaded.robot_poses),
            )
            self.simulation.set_initial_robot_poses(loaded.robot_poses)
            self.map_view.clear_competition_state()
            self._competition_source = loaded.competition_occ_map
            self._competition_world = None
            if loaded.competition_occ_map is not None:
                values = dict(loaded.competition_config or {})
                values["num_robots"] = len(loaded.robot_poses)
                values.setdefault("lidar_range", loaded.sensor_range)
                values.setdefault("start_pose", (15, 15))
                self._competition_config = CompetitionConfig(**{
                    key: value for key, value in values.items()
                    if key in CompetitionConfig.__dataclass_fields__
                })
                self.panel.status.setText(f"COMPETICIÓN CARGADA · {Path(source).stem}")
            self.map_view.zoom = 1.0
            self.map_view.pan.setX(0.0)
            self.map_view.pan.setY(0.0)
            self.panel.show_stopped()
            label = "COMPETICIÓN" if loaded.competition_occ_map is not None else "ESCENARIO"
            self.panel.status.setText(f"{label} CARGADO · {Path(source).name}")
            self.map_view.redraw()
        except (OSError, ValueError, TypeError) as error:
            self.panel.show_error(str(error))

    def _start(self) -> None:
        try:
            name = self.panel.experiment_name.text().strip()
            seed = self.panel.experiment_seed.value()
            self.simulation.configure_experiment(name=name, seed=seed)
            self.settings.setValue("experiment/name", name)
            self.settings.setValue("experiment/seed", seed)
            if self._competition_source is not None:
                if self._competition_thread is not None:
                    self._stop_competition_worker()
                    self.panel.show_stopped()
                    return
                information_gain = create_information_gain(
                    self._information_gain_name, self._information_gain_radius
                )
                if self._competition_policy_path is not None:
                    policy = load_policy(self._competition_policy_path)
                elif self._competition_policy_name == "Next Best View":
                    policy = NextBestViewPolicy(self._utility_weights, information_gain)
                elif self._competition_policy_name == "Adaptive NBV":
                    policy = make_advanced_policy("adaptive")
                elif self._competition_policy_name == "Tuned NBV":
                    policy = make_advanced_policy("tuned")
                elif self._competition_policy_name == "Gain per Cost":
                    policy = make_advanced_policy("gain_per_cost")
                elif self._competition_policy_name == "Coordinated Occlusion":
                    policy = make_advanced_policy("coordinated")
                elif self._competition_policy_name == "Nearest":
                    from Logic.Competition import NearestFrontierPolicy
                    policy = NearestFrontierPolicy()
                elif self._competition_policy_name == "Intent-aware Nearest":
                    policy = make_advanced_policy("intent_nearest")
                elif self._competition_policy_name == "Trajectory Diversified":
                    policy = make_advanced_policy("trajectory_diversified")
                elif self._competition_policy_name == "Recent Trail":
                    policy = make_advanced_policy("recent_trail")
                elif self._competition_policy_name == "Voronoi Nearest":
                    policy = make_advanced_policy("voronoi_nearest")
                elif self._competition_policy_name == "Frontier Reservation":
                    policy = make_advanced_policy("frontier_reservation")
                elif self._competition_policy_name == "Elastic Trajectory":
                    policy = make_advanced_policy("elastic_trajectory")
                elif self._competition_policy_name == "Clearance Utility":
                    policy = make_advanced_policy("clearance_utility")
                elif self._competition_policy_name == "Detour Capped":
                    policy = make_advanced_policy("detour_capped")
                elif self._competition_policy_name == "Soft Intent Nearest":
                    policy = make_advanced_policy("soft_intent_nearest")
                else:
                    policy = WeightedUtilityPolicy(self._utility_weights, information_gain)
                self._competition_world = CompetitionWorld(
                    self._competition_source, self._competition_config, policy,
                    make_return_policy(self._return_policy_name),
                    make_handoff_policy(self._handoff_policy_name)
                )
                self._paused = False
                self.panel.show_running("Competition A*", policy.__class__.__name__)
                self._sync_competition_view()
                self._start_competition_worker()
                return
            if self.simulation.running:
                self.timer.stop()
                self.simulation.reset()
                self._paused = False
                self.panel.show_stopped()
                self.map_view.redraw()
                return
            status = self.simulation.start()
            geometry = self.simulation.robot.geometry
            self.map_view.set_robot_geometry(geometry.length, geometry.width)
            self._paused = False
            self.panel.show_running(status.path_planner, status.objective_assigner)
            self.timer.start()
            self.map_view.redraw()
        except (TypeError, ValueError, NotImplementedError) as error:
            self.panel.show_error(str(error))

    def _export_experiment_json(self) -> None:
        destination, _ = QFileDialog.getSaveFileName(
            self, "Exportar experimento", "experiment.json", "JSON (*.json)"
        )
        if not destination:
            return
        try:
            self.simulation.save_experiment_json(destination)
        except (RuntimeError, OSError, ValueError) as error:
            self.panel.show_error(str(error))

    def _export_experiment_csv(self) -> None:
        destination, _ = QFileDialog.getSaveFileName(
            self, "Exportar métricas", "experiment.csv", "CSV (*.csv)"
        )
        if not destination:
            return
        try:
            self.simulation.export_experiment_csv(destination)
        except (RuntimeError, OSError, ValueError) as error:
            self.panel.show_error(str(error))

    def _toggle_pause(self) -> None:
        if self._competition_world is None and not self.simulation.running:
            return
        self._paused = not self._paused
        if self._competition_worker is not None:
            self._competition_worker.paused = self._paused
            self.panel.show_paused(self._paused)
            return
        if self._paused:
            self.timer.stop()
        else:
            self.timer.start()
        self.panel.show_paused(self._paused)

    @staticmethod
    def _dock(title: str, content: QWidget, name: str) -> QDockWidget:
        dock = QDockWidget(title)
        dock.setObjectName(name)
        dock.setWidget(content)
        dock.setMinimumWidth(260)
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        return dock

    def _open_visual_settings(self, button: QWidget) -> None:
        menu = QMenu(self)
        belief_action = menu.addAction("Belief Map…")
        selected = menu.exec(button.mapToGlobal(button.rect().bottomLeft()))
        if selected is not belief_action:
            return
        VisualSettingsDialog(
            self.belief_controller, self.settings,
            self.map_view.set_robot_style, self,
        ).exec()

    def _visual_changed(self) -> None:
        for occupancy in (-1, 0, 1):
            self.settings.setValue(
                f"visual/belief_{occupancy}",
                self.belief_controller.color_for(occupancy),
            )
        self.map_view.redraw()

    def _move_robot(self, world_x: float, world_y: float) -> None:
        local = self.map_controller.simulation_map.world_to_local((world_x, world_y))
        self.simulation.move_robot(*local)
        self.map_view.redraw()

    def _step(self) -> None:
        try:
            self.simulation.step(0.05)
            tracker = self.simulation.control.safe_tracker
            status = getattr(tracker, "status", None)
            if status is not None and status.emergency:
                self.map_view.set_safety_state("emergency")
            elif status is not None and status.active:
                self.map_view.set_safety_state("active")
            else:
                self.map_view.set_safety_state("clear")
            navigation = self.simulation.navigation_snapshot
            self.map_view.set_navigation_status(
                navigation.state.value,
                navigation.reason,
            )
            metrics = self.simulation.metrics_snapshot
            self.map_view.set_exploration_metrics(
                metrics.outcome.value,
                metrics.coverage,
                metrics.distance_traveled,
                metrics.elapsed_time,
                metrics.goals_reached,
                metrics.replans,
            )
            if not self.simulation.running:
                self.timer.stop()
                self.panel.show_stopped()
            self.map_view.redraw()
        except (TypeError, ValueError) as error:
            self.timer.stop()
            self.simulation.stop()
            self.panel.show_error(str(error))

    def _record_competition_trial(self) -> None:
        world = self._competition_world
        if (world is None or self._competition_source is None
                or self._competition_policy_path is not None):
            return
        environment = self._competition_source.parent.name
        if environment not in ENV_SIZE:
            return
        weights = self._utility_weights
        self._competition_trials.append(TrialResult(
            weights.label, weights.information_gain, weights.travel_cost,
            weights.redundancy, weights.relay_risk, environment,
            ENV_SIZE[environment], world.config.num_robots,
            int(world.config.start_pose[0]), int(world.config.start_pose[1]),
            world.config.max_steps, world.coverage,
        ))

    def _sync_competition_view(self) -> None:
        world = self._competition_world
        if world is None:
            return
        p, ppm = world.config.pd_size, world.config.pixel_per_meter
        poses = tuple((robot.pose[1]/ppm - p/ppm,
                       -(robot.pose[0]/ppm - p/ppm), 0.0) for robot in world.robots)
        self.map_controller.configure_robot_poses(poses)
        if self._competition_map_view == "Reportado a base":
            observed = world.base_obs_map
        elif self._competition_map_view == "Robot 1":
            observed = world.robots[0].combined_obs_map
        else:
            observed = world.live_observation_map()
        self.map_view.set_competition_state(world.occ_map, observed, p, ppm)
        self.map_view.set_base_station((world.base_pose[1]/ppm-p/ppm,
                                        -(world.base_pose[0]/ppm-p/ppm)))
        self.map_view.set_navigation_status(
            f"STEP {world.timestep}/{world.config.max_steps}",
            " · ".join(f"R{r.id}:{r.behavior_mode}" for r in world.robots),
        )
        self.map_view.set_exploration_metrics(
            "COMPETITION", world.coverage, 0.0, float(world.timestep), 0, 0
        )

    def _start_competition_worker(self) -> None:
        if self._competition_world is None:
            return
        thread = QThread(self)
        worker = CompetitionWorker(self._competition_world, self._competition_map_view)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.frame_ready.connect(self._on_competition_frame)
        worker.completed.connect(self._on_competition_completed)
        worker.failed.connect(self._on_competition_failed)
        worker.completed.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        self._competition_thread, self._competition_worker = thread, worker
        thread.start()

    def _stop_competition_worker(self) -> None:
        worker, thread = self._competition_worker, self._competition_thread
        if worker is not None:
            worker.stopped = True
        if thread is not None:
            thread.quit()
            thread.wait(3000)
            thread.deleteLater()
        self._competition_worker = None
        self._competition_thread = None

    def _on_competition_frame(self, frame: CompetitionFrame) -> None:
        world = self._competition_world
        if world is None:
            return
        self._last_competition_frame = frame
        p, ppm = world.config.pd_size, world.config.pixel_per_meter
        poses = tuple((col/ppm-p/ppm, -(row/ppm-p/ppm), 0.0)
                      for row, col in frame.poses)
        self.map_controller.configure_robot_poses(poses)
        self.map_view.set_competition_state(world.occ_map, frame.observed, p, ppm)
        self.map_view.set_navigation_status(
            f"STEP {frame.timestep}/{frame.max_steps}",
            " · ".join(f"R{i+1}:{mode}" for i, mode in enumerate(frame.modes)),
        )
        self.map_view.set_exploration_metrics(
            "COMPETITION", frame.coverage, 0.0, float(frame.timestep), 0, 0
        )

    def _on_competition_completed(self) -> None:
        self._record_competition_trial()
        coverage = self._competition_world.coverage if self._competition_world else 0.0
        self.panel.show_stopped()
        self.panel.status.setText(f"FINAL · COBERTURA BASE {coverage*100:.2f}%")
        self._competition_worker = None
        self._competition_thread = None

    def _on_competition_failed(self, detail: str) -> None:
        self.panel.show_error(detail.splitlines()[-1] if detail else "Error de competición")
        self._competition_worker = None
        self._competition_thread = None

    def _toggle_frontiers(self, visible: bool) -> None:
        self.map_controller.set_show_frontiers(visible)
        self.map_view.redraw()

    def _toggle_clusters(self, visible: bool) -> None:
        self.map_controller.set_show_clusters(visible)
        self.map_view.redraw()

    def _toggle_route(self, visible: bool) -> None:
        self.map_controller.set_show_route(visible)
        self.map_view.redraw()

    def _toggle_obstacles(self, visible: bool) -> None:
        self.map_controller.set_show_obstacles(visible)
        self.map_view.redraw()

    def _change_fov(self, degrees: float) -> None:
        self.simulation.set_sensor_fov(degrees)
        self.map_view.redraw()

    def _change_sensor_radius(self, radius: float) -> None:
        self.simulation.set_sensor_radius(radius)
        self.map_view.redraw()

    def _change_robot_count(self, count: int) -> None:
        status = self.simulation.set_robot_count(count)
        if status.running and not self._paused:
            self.panel.show_running(
                status.path_planner, status.objective_assigner
            )
        self.map_view.redraw()

    def _change_safety_radius(self, radius: float) -> None:
        self.simulation.set_safety_radius(radius)
        self.map_view.set_safety_radius(radius)

    def _change_grid(self, size: float) -> None:
        status = self.simulation.set_grid_size(size)
        if status.running:
            if self._paused:
                self.panel.show_paused(True)
            else:
                self.panel.show_running(
                    status.path_planner, status.objective_assigner
                )
        self.map_view.redraw()

    def closeEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        self._stop_competition_worker()
        self.settings.setValue("workspace/layout", self.saveState())
        self.settings.sync()
        super().closeEvent(event)
