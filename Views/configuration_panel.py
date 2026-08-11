"""Panel Qt para configurar y supervisar la simulación."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QPushButton,
    QHBoxLayout,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from Controllers.belief_map_controller import BeliefMapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.safe_tracker_controller import SafeTrackerController
from Controllers.clustering_controller import ClusteringController
from Controllers.coordination_controller import CoordinationController
from .sensor_preview import SensorPreview


class ConfigurationPanel(QWidget):
    def __init__(
        self,
        path_controller: PathPlannerController,
        objective_controller: ObjectiveAssignController,
        belief_controller: BeliefMapController,
        safe_tracker_controller: SafeTrackerController,
        clustering_controller: ClusteringController,
        coordination_controller: CoordinationController,
        on_start: Callable[[], None],
        on_toggle_frontiers: Callable[[bool], None],
        on_toggle_route: Callable[[bool], None],
        on_toggle_obstacles: Callable[[bool], None],
        on_toggle_clusters: Callable[[bool], None],
        on_change_fov: Callable[[float], None],
        on_change_sensor_radius: Callable[[float], None],
        on_change_safety_radius: Callable[[float], None],
        on_change_grid_size: Callable[[float], None],
        on_belief_style_change: Callable[[], None],
        on_pause: Callable[[], None],
        on_export_json: Callable[[], None],
        on_export_csv: Callable[[], None],
        on_change_robot_count: Callable[[int], None],
        on_load_sim: Callable[[], None],
        on_load_policy: Callable[[], None],
        on_change_policy_weights: Callable[[float, float, float, float], None],
        on_export_competition_table: Callable[[], None],
        on_change_robot_visual_scale: Callable[[float], None],
        on_change_competition_map_view: Callable[[str], None],
        on_change_information_gain: Callable[[str, int], None],
        on_change_competition_policy: Callable[[str], None],
        on_change_return_policy: Callable[[str], None],
        on_change_handoff_policy: Callable[[str], None],
    ) -> None:
        super().__init__()
        self.setObjectName("sidePanel")
        self.setMinimumWidth(310)
        self._path_controller = path_controller
        self._objective_controller = objective_controller
        self._belief_controller = belief_controller
        self._safe_tracker_controller = safe_tracker_controller
        self._clustering_controller = clustering_controller
        self._coordination_controller = coordination_controller
        self._on_toggle_frontiers = on_toggle_frontiers
        self._on_toggle_clusters = on_toggle_clusters
        self._on_start = on_start
        self._on_pause = on_pause

        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 22, 22, 22)
        layout.setSpacing(16)
        title = QLabel("EXPLORATION LAB")
        title.setObjectName("brand")
        subtitle = QLabel("Autonomous robot simulator")
        subtitle.setObjectName("muted")
        layout.addWidget(title)
        layout.addWidget(subtitle)

        self.load_sim_button = QPushButton("CARGAR ARCHIVO .SIM")
        self.load_sim_button.setObjectName("secondaryButton")
        self.load_sim_button.setToolTip("Abrir un escenario robotics_sim_lab.sim")
        layout.addWidget(self.load_sim_button)
        self.load_policy_button = QPushButton("CARGAR POLICY .PY")
        self.load_policy_button.setObjectName("secondaryButton")
        self.load_policy_button.setToolTip("Cargar una clase Policy(BasePolicy)")
        layout.addWidget(self.load_policy_button)

        utility = QGroupBox("POLICY UTILITY")
        utility_layout = QGridLayout(utility)
        self.weight_inputs = []
        for index, (label, value) in enumerate((("wIG", .50), ("wC", .25),
                                                 ("wR", .20), ("wL", .05))):
            row, column = divmod(index, 2)
            selector = QDoubleSpinBox()
            selector.setRange(0.0, 1.0); selector.setSingleStep(0.05)
            selector.setDecimals(2); selector.setValue(value)
            utility_layout.addWidget(QLabel(label), row * 2, column)
            utility_layout.addWidget(selector, row * 2 + 1, column)
            self.weight_inputs.append(selector)
        self.weight_sum = QLabel("SUMA = 1.00")
        self.information_gain_selector = self._combo(
            ("Unknown cells (circular)", "Frontier density",
             "Potential visibility (raycast)")
        )
        self.competition_policy_selector = self._combo(
            ("Weighted Frontier", "Nearest", "Trajectory Diversified",
             "Recent Trail", "Voronoi Nearest", "Frontier Reservation",
             "Elastic Trajectory", "Clearance Utility", "Detour Capped",
             "Intent-aware Nearest",
             "Soft Intent Nearest",
             "Next Best View", "Tuned NBV", "Adaptive NBV",
             "Gain per Cost", "Coordinated Occlusion", "External file")
        )
        self.information_gain_radius = QSpinBox()
        self.information_gain_radius.setRange(3, 100)
        self.information_gain_radius.setValue(25)
        self.information_gain_radius.setSuffix(" px")
        self.return_policy_selector = self._combo(
            ("Periodic", "Efficient Periodic", "Just In Time",
             "Selective Courier", "Value Density", "Deadline",
             "Payload Adaptive", "Link Aware", "Nearest Frontier Return",
             "Gain Sweep Return", "Homeward Sweep Return")
        )
        self.handoff_policy_selector = self._combo(
            ("Closest Progress", "Payload Progress", "Time Saving",
             "Returning Courier", "Link Quality")
        )
        utility_layout.addWidget(QLabel("Policy"), 4, 0, 1, 2)
        utility_layout.addWidget(self.competition_policy_selector, 5, 0, 1, 2)
        utility_layout.addWidget(QLabel("Information gain"), 6, 0, 1, 2)
        utility_layout.addWidget(self.information_gain_selector, 7, 0)
        utility_layout.addWidget(self.information_gain_radius, 7, 1)
        self.export_competition_button = QPushButton("EXPORTAR TABLA EXPERIMENTAL")
        utility_layout.addWidget(QLabel("Return policy"), 8, 0, 1, 2)
        utility_layout.addWidget(self.return_policy_selector, 9, 0, 1, 2)
        utility_layout.addWidget(QLabel("Handoff policy"), 10, 0, 1, 2)
        utility_layout.addWidget(self.handoff_policy_selector, 11, 0, 1, 2)
        utility_layout.addWidget(self.weight_sum, 12, 0, 1, 2)
        utility_layout.addWidget(self.export_competition_button, 13, 0, 1, 2)
        layout.addWidget(utility)

        competition_view = QGroupBox("VISTA DE COMPETICIÓN")
        competition_view_layout = QGridLayout(competition_view)
        self.competition_map_view = self._combo(
            ("Exploración en vivo", "Reportado a base", "Robot 1")
        )
        self.robot_visual_scale = QDoubleSpinBox()
        self.robot_visual_scale.setRange(0.20, 2.0)
        self.robot_visual_scale.setSingleStep(0.05)
        self.robot_visual_scale.setValue(0.55)
        self.robot_visual_scale.setSuffix(" x")
        competition_view_layout.addWidget(QLabel("Mapa"), 0, 0)
        competition_view_layout.addWidget(self.competition_map_view, 1, 0, 1, 2)
        competition_view_layout.addWidget(QLabel("Tamaño de drones"), 2, 0)
        competition_view_layout.addWidget(self.robot_visual_scale, 2, 1)
        layout.addWidget(competition_view)

        algorithms = QGroupBox("ALGORITMOS")
        form = QGridLayout(algorithms)
        self.path_selector = self._combo(path_controller.available_methods)
        self.path_selector.setCurrentText(path_controller.selected_method)
        self.objective_selector = self._combo(objective_controller.available_methods)
        self.objective_selector.setCurrentText(objective_controller.selected_method)
        form.addWidget(QLabel("Planificador"), 0, 0)
        form.addWidget(self.path_selector, 1, 0)
        form.addWidget(QLabel("Asignación de objetivo"), 2, 0)
        form.addWidget(self.objective_selector, 3, 0)
        self.safe_selector = self._combo(safe_tracker_controller.available_methods)
        self.safe_selector.setCurrentText(safe_tracker_controller.selected_method)
        form.addWidget(QLabel("Safe tracker"), 4, 0)
        form.addWidget(self.safe_selector, 5, 0)
        self.clustering_selector = self._combo(
            clustering_controller.available_methods
        )
        self.clustering_selector.setCurrentText(
            clustering_controller.selected_method
        )
        form.addWidget(QLabel("Clustering de fronteras"), 6, 0)
        form.addWidget(self.clustering_selector, 7, 0)
        self.coordination_selector = self._combo(
            coordination_controller.available_methods
        )
        self.coordination_selector.setCurrentText(
            coordination_controller.selected_method
        )
        form.addWidget(QLabel("Coordinación"), 8, 0)
        form.addWidget(self.coordination_selector, 9, 0)
        self.robot_count_selector = QSpinBox()
        self.robot_count_selector.setRange(1, 20)
        self.robot_count_selector.setValue(1)
        form.addWidget(QLabel("Número de robots"), 10, 0)
        form.addWidget(self.robot_count_selector, 11, 0)
        layout.addWidget(algorithms)

        perception = QGroupBox("PERCEPCIÓN")
        grid = QGridLayout(perception)
        self.fov_selector = QDoubleSpinBox()
        self.fov_selector.setRange(1.0, 360.0)
        self.fov_selector.setValue(360.0)
        self.fov_selector.setSuffix("°")
        self.fov_selector.setMaximumWidth(82)
        self.fov_slider = QSlider(Qt.Orientation.Horizontal)
        self.fov_slider.setRange(1, 360)
        self.fov_slider.setValue(360)
        self.sensor_preview = SensorPreview()
        self.sensor_preview.setFixedSize(46, 46)
        self.grid_selector = QDoubleSpinBox()
        self.grid_selector.setRange(0.1, 5.0)
        self.grid_selector.setSingleStep(0.1)
        self.grid_selector.setValue(1.0)
        self.grid_selector.setSuffix(" m")
        self.grid_selector.setMaximumWidth(82)
        self.grid_slider = QSlider(Qt.Orientation.Horizontal)
        self.grid_slider.setRange(1, 50)
        self.grid_slider.setValue(10)
        grid.setColumnStretch(0, 1)
        grid.addWidget(QLabel("Campo de visión"), 0, 0, 1, 3)
        grid.addWidget(self.fov_slider, 1, 0)
        grid.addWidget(self.sensor_preview, 1, 1)
        grid.addWidget(self.fov_selector, 1, 2)
        grid.addWidget(QLabel("Resolución del grid"), 2, 0, 1, 3)
        grid.addWidget(self.grid_slider, 3, 0)
        grid.addWidget(self.grid_selector, 3, 2)
        self.radius_selector = QDoubleSpinBox()
        self.radius_selector.setRange(0.5, 30.0)
        self.radius_selector.setSingleStep(0.5)
        self.radius_selector.setValue(10.0)
        self.radius_selector.setSuffix(" m")
        self.radius_selector.setMaximumWidth(82)
        self.radius_slider = QSlider(Qt.Orientation.Horizontal)
        self.radius_slider.setRange(5, 300)
        self.radius_slider.setValue(100)
        grid.addWidget(QLabel("Distancia del sensor"), 4, 0, 1, 3)
        grid.addWidget(self.radius_slider, 5, 0)
        grid.addWidget(self.radius_selector, 5, 2)
        layout.addWidget(perception)

        safety = QGroupBox("SEGURIDAD")
        safety_layout = QGridLayout(safety)
        self.safety_radius_selector = QDoubleSpinBox()
        self.safety_radius_selector.setRange(0.0, 5.0)
        self.safety_radius_selector.setSingleStep(0.05)
        self.safety_radius_selector.setValue(0.20)
        self.safety_radius_selector.setSuffix(" m")
        self.safety_radius_selector.setMaximumWidth(82)
        self.safety_radius_slider = QSlider(Qt.Orientation.Horizontal)
        self.safety_radius_slider.setRange(0, 100)
        self.safety_radius_slider.setValue(4)
        safety_layout.addWidget(QLabel("Radio adicional"), 0, 0, 1, 2)
        safety_layout.addWidget(self.safety_radius_slider, 1, 0)
        safety_layout.addWidget(self.safety_radius_selector, 1, 1)
        layout.addWidget(safety)

        layers = QGroupBox("CAPAS")
        layer_layout = QVBoxLayout(layers)
        self.frontiers = QCheckBox("Fronteras en tiempo real")
        self.route = QCheckBox("Waypoints y ruta")
        self.clusters = QCheckBox("Mostrar clusters por color")
        self.clusters.setEnabled(clustering_controller.clustering_enabled)
        self.obstacles = QCheckBox("Mostrar obstáculos")
        self.obstacles.setChecked(True)
        layer_layout.addWidget(self.obstacles)
        layer_layout.addWidget(self.frontiers)
        layer_layout.addWidget(self.clusters)
        layer_layout.addWidget(self.route)
        layout.addWidget(layers)

        experiment = QGroupBox("EXPERIMENTO")
        experiment_layout = QGridLayout(experiment)
        self.experiment_name = QLineEdit("exploration-run")
        self.experiment_seed = QSpinBox()
        self.experiment_seed.setRange(0, 2_147_483_647)
        self.experiment_seed.setValue(0)
        self.export_json_button = QPushButton("Exportar JSON")
        self.export_csv_button = QPushButton("Exportar CSV")
        experiment_layout.addWidget(QLabel("Nombre"), 0, 0)
        experiment_layout.addWidget(self.experiment_name, 1, 0, 1, 2)
        experiment_layout.addWidget(QLabel("Semilla"), 2, 0)
        experiment_layout.addWidget(self.experiment_seed, 2, 1)
        experiment_layout.addWidget(self.export_json_button, 3, 0)
        experiment_layout.addWidget(self.export_csv_button, 3, 1)
        layout.addWidget(experiment)

        layout.addStretch()

        self.status = QLabel("LISTO PARA INICIAR")
        self.status.setObjectName("statusPill")
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status)
        self.start_button = QPushButton("INICIAR SIMULACIÓN")
        self.start_button.setObjectName("primaryButton")
        self.start_button.setMinimumHeight(46)
        layout.addWidget(self.start_button)
        self.pause_button = QPushButton("PAUSAR")
        self.pause_button.setObjectName("secondaryButton")
        self.pause_button.setMinimumHeight(40)
        self.pause_button.setEnabled(False)
        layout.addWidget(self.pause_button)

        self.path_selector.currentTextChanged.connect(path_controller.select)
        self.objective_selector.currentTextChanged.connect(objective_controller.select)
        self.safe_selector.currentTextChanged.connect(safe_tracker_controller.select)
        self.clustering_selector.currentTextChanged.connect(
            self._select_clustering
        )
        self.coordination_selector.currentTextChanged.connect(
            coordination_controller.select
        )
        self.fov_slider.valueChanged.connect(self.fov_selector.setValue)
        self.fov_selector.valueChanged.connect(lambda value: self.fov_slider.setValue(round(value)))
        self.fov_selector.valueChanged.connect(on_change_fov)
        self.fov_selector.valueChanged.connect(self.sensor_preview.set_fov)
        self.grid_slider.valueChanged.connect(lambda value: self.grid_selector.setValue(value / 10.0))
        self.grid_selector.valueChanged.connect(lambda value: self.grid_slider.setValue(round(value * 10)))
        self.grid_selector.valueChanged.connect(on_change_grid_size)
        self.grid_selector.valueChanged.connect(self.sensor_preview.set_grid_size)
        self.radius_slider.valueChanged.connect(
            lambda value: self.radius_selector.setValue(value / 10.0)
        )
        self.radius_selector.valueChanged.connect(
            lambda value: self.radius_slider.setValue(round(value * 10))
        )
        self.radius_selector.valueChanged.connect(on_change_sensor_radius)
        self.safety_radius_slider.valueChanged.connect(
            lambda value: self.safety_radius_selector.setValue(value * 0.05)
        )
        self.safety_radius_selector.valueChanged.connect(
            lambda value: self.safety_radius_slider.setValue(round(value / 0.05))
        )
        self.safety_radius_selector.valueChanged.connect(on_change_safety_radius)
        self.frontiers.toggled.connect(self._toggle_frontiers)
        self.clusters.toggled.connect(self._toggle_clusters)
        self.route.toggled.connect(on_toggle_route)
        self.obstacles.toggled.connect(on_toggle_obstacles)
        self.start_button.clicked.connect(self._start)
        self.pause_button.clicked.connect(self._on_pause)
        self.export_json_button.clicked.connect(on_export_json)
        self.export_csv_button.clicked.connect(on_export_csv)
        self.robot_count_selector.valueChanged.connect(on_change_robot_count)
        self.load_sim_button.clicked.connect(on_load_sim)
        self.load_policy_button.clicked.connect(on_load_policy)
        for selector in self.weight_inputs:
            selector.valueChanged.connect(lambda _value: self._weights_changed(on_change_policy_weights))
        self.export_competition_button.clicked.connect(on_export_competition_table)
        self.robot_visual_scale.valueChanged.connect(on_change_robot_visual_scale)
        self.competition_map_view.currentTextChanged.connect(on_change_competition_map_view)
        self.information_gain_selector.currentTextChanged.connect(
            lambda name: on_change_information_gain(name, self.information_gain_radius.value()))
        self.information_gain_radius.valueChanged.connect(
            lambda radius: on_change_information_gain(self.information_gain_selector.currentText(), radius))
        self.competition_policy_selector.currentTextChanged.connect(on_change_competition_policy)
        self.return_policy_selector.currentTextChanged.connect(on_change_return_policy)
        self.handoff_policy_selector.currentTextChanged.connect(on_change_handoff_policy)

    def _weights_changed(self, callback) -> None:
        values = tuple(selector.value() for selector in self.weight_inputs)
        total = sum(values)
        self.weight_sum.setText(f"SUMA = {total:.2f}" + (" OK" if abs(total-1.0) < 1e-9 else " · debe ser 1.00"))
        self.weight_sum.setStyleSheet("color: #86efac" if abs(total-1.0) < 1e-9 else "color: #fca5a5")
        callback(*values)

    def apply_sim_configuration(self, *, grid_size: float, sensor_range: float,
                                sensor_fov: float, safety_radius: float,
                                robot_count: int) -> None:
        """Refleja en la interfaz los parámetros importados."""
        self.grid_selector.setValue(min(5.0, max(0.1, grid_size)))
        self.radius_selector.setValue(min(30.0, max(0.5, sensor_range)))
        self.fov_selector.setValue(min(360.0, max(1.0, sensor_fov)))
        self.safety_radius_selector.setValue(min(5.0, max(0.0, safety_radius)))
        self.robot_count_selector.setValue(min(20, max(1, robot_count)))

    @staticmethod
    def _combo(values: tuple[str, ...]) -> QComboBox:
        combo = QComboBox()
        combo.addItems(values)
        return combo

    def _start(self) -> None:
        self._path_controller.select(self.path_selector.currentText())
        self._objective_controller.select(self.objective_selector.currentText())
        self._on_start()

    def _select_clustering(self, name: str) -> None:
        self._clustering_controller.select(name)
        enabled = self._clustering_controller.clustering_enabled
        self.clusters.setEnabled(enabled)
        if not enabled:
            self.clusters.setChecked(False)

    def _toggle_frontiers(self, visible: bool) -> None:
        if visible and self.clusters.isChecked():
            self.clusters.setChecked(False)
        self._on_toggle_frontiers(visible)

    def _toggle_clusters(self, visible: bool) -> None:
        if visible and self.frontiers.isChecked():
            self.frontiers.setChecked(False)
        self._on_toggle_clusters(visible)

    def show_running(self, path_planner: str, objective_assigner: str) -> None:
        self.status.setText(f"ACTIVO  ·  {path_planner}  ·  {objective_assigner}")
        self.status.setProperty("running", True)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.start_button.setText("REINICIAR SIMULACIÓN")
        self.pause_button.setEnabled(True)
        self.pause_button.setText("PAUSAR")

    def show_paused(self, paused: bool) -> None:
        self.status.setText("EN PAUSA" if paused else "SIMULACIÓN ACTIVA")
        self.status.setProperty("running", not paused)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.pause_button.setText("REANUDAR" if paused else "PAUSAR")

    def show_stopped(self) -> None:
        self.status.setText("LISTO PARA INICIAR")
        self.status.setProperty("running", False)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.start_button.setText("INICIAR SIMULACIÓN")
        self.pause_button.setText("PAUSAR")
        self.pause_button.setEnabled(False)

    def show_error(self, message: str) -> None:
        self.status.setText(f"ERROR  ·  {message}")
        self.status.setProperty("running", False)
