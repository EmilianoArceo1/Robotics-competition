"""Lienzo Qt antialias para visualizar el mundo y la exploración."""

from __future__ import annotations

from math import ceil, cos, degrees, floor, hypot, sin

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Logic.Map.grid_geometry import GridCell


class MapView(QWidget):
    def __init__(self, controller: MapController, belief: BeliefMapController, on_robot_moved=None) -> None:
        super().__init__()
        self.controller = controller
        self.belief = belief
        self.zoom = 1.0
        self.pan = QPointF(0.0, 0.0)
        self._drag_position: QPointF | None = None
        self.robot_length = 0.50
        self.robot_width = 0.35
        self.safety_radius = 0.20
        self.safety_state = "clear"
        self.navigation_status = "IDLE · waiting for route"
        self.metrics_status = "COBERTURA  0.0%     DISTANCIA  0.00 m"
        self.metrics_detail = "TIEMPO  0.0 s     OBJETIVOS  0     REPLANES  0"
        self.robot_style = "Círculo"
        self.robot_visual_scale = 0.55
        self.on_robot_moved = on_robot_moved
        self._dragging_robot = False
        self._robot_selected = False
        self._scale = 1.0
        self._scene_center = (0.0, 0.0)
        self._competition_image: QImage | None = None
        self._competition_extent: tuple[float, float, float, float] | None = None
        self._competition_rgb = None
        self._competition_observed = None
        self._competition_ground_truth = None
        self._base_station_position: tuple[float, float] | None = None
        self.setMinimumSize(520, 420)
        self.setObjectName("mapCanvas")
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def set_robot_geometry(self, length: float, width: float) -> None:
        self.robot_length = float(length)
        self.robot_width = float(width)
        self.update()

    def set_robot_style(self, style: str) -> None:
        self.robot_style = style
        self.update()

    def set_robot_visual_scale(self, scale: float) -> None:
        self.robot_visual_scale = min(2.0, max(0.2, float(scale)))
        self.update()

    def set_base_station(self, position: tuple[float, float] | None) -> None:
        self._base_station_position = position
        self.update()

    def set_safety_radius(self, radius: float) -> None:
        self.safety_radius = float(radius)
        self.update()

    def set_safety_state(self, state: str) -> None:
        self.safety_state = state
        self.update()

    def set_navigation_status(self, state: str, reason: str) -> None:
        self.navigation_status = f"{state} · {reason}"
        self.update()

    def set_exploration_metrics(
        self,
        outcome: str,
        coverage: float,
        distance: float,
        elapsed: float,
        goals: int,
        replans: int,
    ) -> None:
        self.metrics_status = (
            f"{outcome}     COBERTURA  {coverage * 100.0:.1f}%     "
            f"DISTANCIA  {distance:.2f} m"
        )
        self.metrics_detail = (
            f"TIEMPO  {elapsed:.1f} s     OBJETIVOS  {goals}     "
            f"REPLANES  {replans}"
        )
        self.update()

    def redraw(self) -> None:
        self.update()

    def set_competition_state(self, ground_truth, observed, padding: int,
                              pixels_per_meter: int) -> None:
        """Construye la capa de conocimiento exactamente desde 0/0.5/1."""
        import numpy as np
        p = int(padding)
        gt = ground_truth[p:-p, p:-p]
        obs = observed[p:-p, p:-p]
        rebuild = (self._competition_rgb is None
                   or self._competition_observed is None
                   or self._competition_observed.shape != obs.shape)
        if rebuild:
            self._competition_rgb = np.empty((*gt.shape, 3), dtype=np.uint8)
            self._competition_observed = np.full(obs.shape, np.nan)
            self._competition_ground_truth = gt.copy()
            self._competition_image = QImage(
                self._competition_rgb.data, gt.shape[1], gt.shape[0],
                self._competition_rgb.strides[0], QImage.Format.Format_RGB888,
            )
        changed = obs != self._competition_observed
        if np.any(changed):
            rgb = self._competition_rgb
            rgb[changed] = (18, 29, 47)
            rgb[changed & (obs == 0)] = (42, 122, 208)
            rgb[changed & (obs == 1)] = (225, 232, 249)
            rgb[changed & (gt == 1) & (obs == 0.5)] = (35, 50, 72)
            self._competition_observed[changed] = obs[changed]
        self._competition_extent = (0.0, -gt.shape[0] / pixels_per_meter,
                                    gt.shape[1] / pixels_per_meter, 0.0)
        self.update()

    def clear_competition_state(self) -> None:
        self._competition_image = None
        self._competition_extent = None
        self._competition_rgb = None
        self._competition_observed = None
        self._competition_ground_truth = None
        self._base_station_position = None

    def wheelEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        factor = 1.12 if event.angleDelta().y() > 0 else 1 / 1.12
        self.zoom = min(4.0, max(0.45, self.zoom * factor))
        self.update()

    def mousePressEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_position = event.position()
            robot = self._world_to_screen(self.controller.snapshot().robot_position)
            self._dragging_robot = (
                (event.position() - robot).manhattanLength()
                <= max(14.0, self.robot_length * self._scale)
            )
            self._robot_selected = self._dragging_robot
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if self._drag_position is not None:
            if self._dragging_robot and self.on_robot_moved is not None:
                world = self._screen_to_world(event.position())
                self.on_robot_moved(world[0], world[1])
            else:
                self.pan += event.position() - self._drag_position
            self._drag_position = event.position()
            self.update()

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        self._drag_position = None
        self._dragging_robot = False
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def paintEvent(self, _event) -> None:  # type: ignore[no-untyped-def]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#08111f"))
        min_x, min_y, max_x, max_y = self.controller.scene_bounds(2.0)
        base = min(
            (self.width() - 80) / max(max_x - min_x, 1.0),
            (self.height() - 100) / max(max_y - min_y, 1.0),
        )
        scale = base * self.zoom
        center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2
        self._scale = scale
        self._scene_center = center_x, center_y

        def point(coordinate: tuple[float, float]) -> QPointF:
            return QPointF(
                self.width() / 2 + (coordinate[0] - center_x) * scale,
                self.height() / 2 - (coordinate[1] - center_y) * scale,
            ) + self.pan

        self._draw_grid(painter, point, min_x, min_y, max_x, max_y)
        snapshot = self.controller.snapshot()
        if self._competition_image is not None and self._competition_extent is not None:
            x_min, y_min, x_max, y_max = self._competition_extent
            painter.drawImage(QRectF(point((x_min, y_max)), point((x_max, y_min))).normalized(),
                              self._competition_image)
        elif self.controller.show_obstacles:
            self._draw_obstacles(
                painter, point, snapshot.obstacles,
                scale * snapshot.obstacle_size,
            )
        if self._competition_image is None:
            self._draw_belief(painter, point, scale)
        self._draw_fov(painter, point, snapshot.sensor_visibility)
        self._draw_route(painter, point, snapshot.route, snapshot.active_waypoint_index)
        self._draw_frontiers(painter, point, snapshot.frontiers)
        self._draw_clusters(painter, point, snapshot.frontier_clusters, scale)
        for robot in snapshot.robots:
            self._draw_robot(
                painter, point(robot.position), robot.heading, scale,
                primary=robot.controllable, label=robot.robot_id,
            )
        if self._base_station_position is not None:
            self._draw_base_station(painter, point(self._base_station_position), scale)
        self._draw_hud(
            painter, snapshot.robot_position, len(snapshot.obstacles),
            len(snapshot.robots), scale,
        )

    def _world_to_screen(self, coordinate: tuple[float, float]) -> QPointF:
        center_x, center_y = self._scene_center
        return QPointF(
            self.width() / 2 + (coordinate[0] - center_x) * self._scale,
            self.height() / 2 - (coordinate[1] - center_y) * self._scale,
        ) + self.pan

    def _screen_to_world(self, point: QPointF) -> tuple[float, float]:
        center_x, center_y = self._scene_center
        adjusted = point - self.pan
        return (
            center_x + (adjusted.x() - self.width() / 2) / self._scale,
            center_y - (adjusted.y() - self.height() / 2) / self._scale,
        )

    def _draw_grid(self, painter, point, min_x, min_y, max_x, max_y) -> None:
        cell_size = self.belief.grid_size
        geometry = self.belief.geometry
        painter.setPen(QPen(QColor("#16243a"), 1))
        # Las coordenadas del Belief Map representan centros de celda;
        # las líneas del fondo deben coincidir con sus bordes (+/- media celda).
        first_column = floor(min_x / cell_size - 0.5)
        last_column = ceil(max_x / cell_size - 0.5)
        first_row = floor(min_y / cell_size - 0.5)
        last_row = ceil(max_y / cell_size - 0.5)
        for column in range(first_column, last_column + 1):
            _, x, _, _ = geometry.cell_bounds(GridCell(column, 0))
            painter.drawLine(point((x, min_y)), point((x, max_y)))
        for row in range(first_row, last_row + 1):
            _, _, _, y = geometry.cell_bounds(GridCell(0, row))
            painter.drawLine(point((min_x, y)), point((max_x, y)))
        painter.setPen(QPen(QColor("#314158"), 1.5))
        painter.drawLine(point((0, min_y)), point((0, max_y)))
        painter.drawLine(point((min_x, 0)), point((max_x, 0)))

    def _draw_belief(self, painter, point, scale: float) -> None:
        """Rellena tiles con bordes compartidos, sin costuras subpíxel."""
        geometry = self.belief.geometry
        painter.save()
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        painter.setPen(Qt.PenStyle.NoPen)
        for cell in self.belief.cells:
            x, y = cell.coordinate
            grid_cell = geometry.world_to_cell(x, y)
            x_min, x_max, y_min, y_max = geometry.cell_bounds(grid_cell)
            top_left = point((x_min, y_max))
            bottom_right = point((x_max, y_min))
            rectangle = QRectF(top_left, bottom_right).normalized()
            color = QColor(self.belief.color_for(cell.occupancy))
            color.setAlpha(115 if cell.occupancy != 1 else 180)
            painter.fillRect(rectangle, color)
        painter.restore()

    @staticmethod
    def _draw_fov(painter, point, visibility) -> None:
        if len(visibility) < 3:
            return
        polygon = QPolygonF([point(value) for value in visibility])
        painter.setPen(QPen(QColor("#38bdf8"), 1.5))
        painter.setBrush(QColor(14, 165, 233, 38))
        painter.drawPolygon(polygon)

    @staticmethod
    def _draw_obstacles(painter, point, obstacles, size: float) -> None:
        painter.setPen(QPen(QColor("#fb7185"), 2))
        painter.setBrush(QColor("#4c1d2b"))
        for obstacle in obstacles:
            center = point(obstacle)
            rectangle = QRectF(
                center.x() - size / 2,
                center.y() - size / 2,
                size,
                size,
            )
            if size < 8.0:
                painter.drawRect(rectangle)
            else:
                radius = min(5.0, size * 0.12)
                painter.drawRoundedRect(rectangle, radius, radius)

    @staticmethod
    def _draw_frontiers(painter, point, frontiers) -> None:
        painter.setPen(QPen(QColor("#fb923c"), 2))
        painter.setBrush(QColor("#fed7aa"))
        for frontier in frontiers:
            center = point(frontier)
            painter.drawEllipse(center, 4.5, 4.5)

    @staticmethod
    def _draw_clusters(painter, point, clusters, scale: float) -> None:
        palette = (
            "#22d3ee", "#a78bfa", "#f472b6", "#facc15",
            "#4ade80", "#fb923c", "#60a5fa", "#e879f9",
        )
        radius = max(3.5, min(7.0, scale * 0.14))
        for index, cluster in enumerate(clusters):
            color = QColor(palette[index % len(palette)])
            painter.setPen(QPen(color.lighter(135), 1.5))
            painter.setBrush(color)
            for coordinate in cluster:
                center = point(coordinate)
                painter.drawEllipse(center, radius, radius)

    @staticmethod
    def _draw_route(painter, point, route, active: int) -> None:
        if not route:
            return
        path = QPainterPath(point(route[0]))
        for coordinate in route[1:]:
            path.lineTo(point(coordinate))
        pen = QPen(QColor("#a78bfa"), 3)
        pen.setDashPattern([5, 3])
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPath(path)
        for index, coordinate in enumerate(route):
            center = point(coordinate)
            painter.setBrush(QColor("#facc15" if index == active else "#8b5cf6"))
            painter.setPen(QPen(QColor("#ffffff"), 1))
            painter.drawEllipse(center, 6 if index == active else 4, 6 if index == active else 4)

    def _draw_robot(
        self, painter, center: QPointF, heading: float, scale: float,
        *, primary: bool = True, label: str = "",
    ) -> None:
        visual_scale = self.robot_visual_scale
        half_length = self.robot_length * scale * visual_scale / 2.0
        half_width = self.robot_width * scale * visual_scale / 2.0
        forward = QPointF(cos(heading), -sin(heading))
        lateral = QPointF(sin(heading), cos(heading))
        polygon = QPolygonF(
            (
                center + forward * half_length + lateral * half_width,
                center + forward * half_length - lateral * half_width,
                center - forward * half_length - lateral * half_width,
                center - forward * half_length + lateral * half_width,
            )
        )
        clearance = hypot(self.robot_length / 2.0, self.robot_width / 2.0)
        clearance = (clearance + self.safety_radius) * scale
        safety_color = {
            "active": QColor(250, 204, 21, 210),
            "emergency": QColor(239, 68, 68, 230),
        }.get(self.safety_state, QColor(34, 197, 94, 150))
        safety_pen = QPen(safety_color, 2.0 if self.safety_state != "clear" else 1.5)
        safety_pen.setDashPattern([4, 3])
        if primary:
            painter.setPen(safety_pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(center, clearance, clearance)
        if primary and self._robot_selected:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(0, 0, 0, 85))
            painter.drawEllipse(center + QPointF(4, 7), half_length * 1.25, half_width * 1.5)
        painter.setPen(QPen(QColor("#bae6fd"), 2))
        painter.setBrush(QColor("#0284c7"))
        if self.robot_style == "Círculo":
            painter.drawEllipse(center, max(half_length, half_width), max(half_length, half_width))
        elif self.robot_style == "Dron":
            drone_pen = QPen(QColor("#cbd5e1"), max(1.5, scale * 0.035))
            drone_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(drone_pen)
            arm_forward = half_length * 0.82
            arm_lateral = half_width * 0.92
            rotors = (
                center + forward * arm_forward + lateral * arm_lateral,
                center + forward * arm_forward - lateral * arm_lateral,
                center - forward * arm_forward + lateral * arm_lateral,
                center - forward * arm_forward - lateral * arm_lateral,
            )
            for rotor in rotors:
                painter.drawLine(center, rotor)
            rotor_radius = max(1.5, scale * 0.065 * visual_scale)
            painter.setPen(QPen(QColor("#f8fafc"), max(1.5, scale * 0.025)))
            painter.setBrush(QColor("#2563eb"))
            for rotor in rotors:
                painter.drawEllipse(rotor, rotor_radius, rotor_radius)
            body_length = max(3.5, scale * 0.20 * visual_scale)
            body_width = max(3.0, scale * 0.16 * visual_scale)
            painter.setPen(QPen(QColor("#f8fafc"), max(1.5, scale * 0.025)))
            painter.setBrush(QColor("#0284c7"))
            painter.save()
            painter.translate(center)
            painter.rotate(-degrees(heading))
            painter.drawRoundedRect(
                QRectF(
                    -body_length / 2,
                    -body_width / 2,
                    body_length,
                    body_width,
                ),
                max(2.0, scale * 0.035),
                max(2.0, scale * 0.035),
            )
            painter.restore()
        else:
            painter.drawRoundedRect(polygon.boundingRect(), 4, 4)
            painter.setPen(QPen(QColor("#172554"), 3))
            painter.drawLine(polygon[0], polygon[1])
            painter.drawLine(polygon[2], polygon[3])
        end = center + forward * half_length
        if self.robot_style != "Dron":
            heading_pen = QPen(QColor("#f8fafc"), 3)
            heading_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(heading_pen)
            painter.drawLine(center, end)
        if label:
            painter.setPen(QColor("#e2e8f0" if primary else "#94a3b8"))
            painter.setFont(QFont("Segoe UI", 7, QFont.Weight.DemiBold))
            painter.drawText(
                QRectF(center.x() - 35, center.y() + half_width + 5, 70, 16),
                Qt.AlignmentFlag.AlignCenter,
                label,
            )

    @staticmethod
    def _draw_base_station(painter, center: QPointF, scale: float) -> None:
        radius = max(3.0, min(7.0, scale * 0.09))
        painter.setPen(QPen(QColor("#fef08a"), 2))
        painter.setBrush(QColor("#f59e0b"))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QColor("#fde68a"))
        painter.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
        painter.drawText(QRectF(center.x()-48, center.y()+radius+3, 96, 16),
                         Qt.AlignmentFlag.AlignCenter, "BASE STATION")

    def _draw_hud(
        self, painter, position, obstacle_count: int,
        robot_count: int, scale: float,
    ) -> None:
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(15, 23, 42, 220))
        painter.drawRoundedRect(QRectF(18, 18, 410, 128), 10, 10)
        painter.setPen(QColor("#e2e8f0"))
        painter.setFont(QFont("Segoe UI", 10, QFont.Weight.DemiBold))
        painter.drawText(34, 43, f"POSE  {position[0]:+.2f}, {position[1]:+.2f}")
        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Segoe UI", 9))
        painter.drawText(
            34, 65,
            f"ROBOTS  {robot_count}     OBSTÁCULOS  {obstacle_count}     "
            f"ZOOM  {self.zoom:.2f}x",
        )
        painter.setPen(QColor("#7dd3fc"))
        painter.drawText(34, 87, self.navigation_status)
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(34, 109, self.metrics_status)
        painter.setPen(QColor("#94a3b8"))
        painter.drawText(34, 131, self.metrics_detail)
