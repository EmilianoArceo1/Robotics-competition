"""Ventana principal profesional basada en Qt 6."""

from __future__ import annotations

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtWidgets import QDockWidget, QMainWindow, QMenu, QScrollArea, QWidget

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.objective_assign_controller import ObjectiveAssignController
from Controllers.path_planner_controller import PathPlannerController
from Controllers.simulation_controller import SimulationController
from Controllers.safe_tracker_controller import SafeTrackerController
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
QComboBox, QDoubleSpinBox { background: #17243a; border: 1px solid #334765; border-radius: 7px;
  padding: 7px; color: #f1f5f9; min-height: 22px; }
QComboBox:hover, QDoubleSpinBox:hover { border-color: #38bdf8; }
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
        self.map_controller = map_controller or MapController()
        self.belief_controller = BeliefMapController(self.map_controller.simulation_map)
        for occupancy in (-1, 0, 1):
            saved = self.settings.value(f"visual/belief_{occupancy}")
            if saved:
                self.belief_controller.set_color(occupancy, str(saved))
        self.path_controller = PathPlannerController()
        self.objective_controller = ObjectiveAssignController()
        self.safe_tracker_controller = SafeTrackerController()
        self.simulation = SimulationController(
            self.map_controller, self.path_controller, self.objective_controller,
            self.belief_controller, self.safe_tracker_controller,
        )
        self.map_view = MapView(
            self.map_controller, self.belief_controller, self._move_robot
        )
        self.map_view.set_robot_style(str(self.settings.value("visual/robot", "Círculo")))
        self.panel = ConfigurationPanel(
            self.path_controller, self.objective_controller, self.belief_controller,
            self.safe_tracker_controller,
            self._start, self._toggle_frontiers, self._toggle_route,
            self._toggle_obstacles,
            self._change_fov, self._change_sensor_radius,
            self._change_safety_radius,
            self._change_grid, self._visual_changed,
            self._toggle_pause,
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

    def _start(self) -> None:
        try:
            if self.simulation.running:
                self.timer.stop()
                self.simulation.start()
                self.simulation.stop()
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

    def _toggle_pause(self) -> None:
        if not self.simulation.running:
            return
        self._paused = not self._paused
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
            self.map_view.redraw()
        except (TypeError, ValueError) as error:
            self.timer.stop()
            self.simulation.stop()
            self.panel.show_error(str(error))

    def _toggle_frontiers(self, visible: bool) -> None:
        self.map_controller.set_show_frontiers(visible)
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
        self.settings.setValue("workspace/layout", self.saveState())
        self.settings.sync()
        super().closeEvent(event)
