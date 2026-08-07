"""Previsualización del FOV y de la resolución configurada."""

from math import cos, radians, sin

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget


class SensorPreview(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.fov = 360.0
        self.grid_size = 1.0
        self.setMinimumSize(40, 40)

    def set_fov(self, value: float) -> None:
        self.fov = float(value)
        self.update()

    def set_grid_size(self, value: float) -> None:
        self.grid_size = float(value)
        self.update()

    def paintEvent(self, _event) -> None:  # type: ignore[no-untyped-def]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101b2d"))
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = min(self.width(), self.height()) * 0.38
        painter.setPen(QPen(QColor("#334765"), 0.8))
        painter.setBrush(QColor("#0b1220"))
        painter.drawEllipse(center, radius, radius)
        start = -self.fov / 2
        path = QPainterPath(center)
        if self.fov >= 359.9:
            path.addEllipse(center, radius, radius)
        else:
            path.lineTo(center + QPointF(cos(radians(start)) * radius, -sin(radians(start)) * radius))
            for step in range(1, 65):
                angle = start + self.fov * step / 64
                path.lineTo(center + QPointF(cos(radians(angle)) * radius, -sin(radians(angle)) * radius))
            path.closeSubpath()
        painter.setPen(QPen(QColor("#38bdf8"), 1.4))
        painter.setBrush(QColor(14, 165, 233, 65))
        painter.drawPath(path)
        painter.setBrush(QColor("#0284c7"))
        painter.drawEllipse(center, 3.5, 3.5)
        painter.setPen(QColor("#94a3b8"))
