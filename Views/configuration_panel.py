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
    QPushButton,
    QHBoxLayout,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from Controllers.belief_map_controller import BeliefMapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.safe_tracker_controller import SafeTrackerController
from .sensor_preview import SensorPreview


class ConfigurationPanel(QWidget):
    def __init__(
        self,
        path_controller: PathPlannerController,
        objective_controller: ObjectiveAssignController,
        belief_controller: BeliefMapController,
        safe_tracker_controller: SafeTrackerController,
        on_start: Callable[[], None],
        on_toggle_frontiers: Callable[[bool], None],
        on_toggle_route: Callable[[bool], None],
        on_toggle_obstacles: Callable[[bool], None],
        on_change_fov: Callable[[float], None],
        on_change_sensor_radius: Callable[[float], None],
        on_change_safety_radius: Callable[[float], None],
        on_change_grid_size: Callable[[float], None],
        on_belief_style_change: Callable[[], None],
        on_pause: Callable[[], None],
    ) -> None:
        super().__init__()
        self.setObjectName("sidePanel")
        self.setMinimumWidth(310)
        self._path_controller = path_controller
        self._objective_controller = objective_controller
        self._belief_controller = belief_controller
        self._safe_tracker_controller = safe_tracker_controller
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
        self.obstacles = QCheckBox("Mostrar obstáculos")
        self.obstacles.setChecked(True)
        layer_layout.addWidget(self.obstacles)
        layer_layout.addWidget(self.frontiers)
        layer_layout.addWidget(self.route)
        layout.addWidget(layers)

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
        self.frontiers.toggled.connect(on_toggle_frontiers)
        self.route.toggled.connect(on_toggle_route)
        self.obstacles.toggled.connect(on_toggle_obstacles)
        self.start_button.clicked.connect(self._start)
        self.pause_button.clicked.connect(self._on_pause)

    @staticmethod
    def _combo(values: tuple[str, ...]) -> QComboBox:
        combo = QComboBox()
        combo.addItems(values)
        return combo

    def _start(self) -> None:
        self._path_controller.select(self.path_selector.currentText())
        self._objective_controller.select(self.objective_selector.currentText())
        self._on_start()

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
