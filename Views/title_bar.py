"""Barra de título personalizada para la ventana sin marco."""

from PySide6.QtCore import QPoint, Qt
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget


class TitleBar(QWidget):
    def __init__(self, window: QWidget, open_settings) -> None:
        super().__init__(window)
        self._window = window
        self._drag_origin: QPoint | None = None
        self.setObjectName("titleBar")
        self.setFixedHeight(44)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 0, 8, 0)
        layout.setSpacing(4)
        mark, title = QLabel("◈"), QLabel("EXPLORATION LAB  /  ROBOT SIMULATOR")
        mark.setObjectName("windowMark")
        title.setObjectName("windowTitle")
        layout.addWidget(mark)
        layout.addWidget(title)
        layout.addStretch()
        settings = QPushButton("⚙")
        settings.setObjectName("windowButton")
        settings.setToolTip("Configurar apariencia")
        settings.clicked.connect(lambda: open_settings(settings))
        layout.addWidget(settings)
        for text, name, tip, callback in (
            ("—", "windowButton", "Minimizar", window.showMinimized),
            ("□", "windowButton", "Maximizar / restaurar", self._toggle_maximized),
            ("×", "closeButton", "Cerrar", window.close),
        ):
            button = QPushButton(text)
            button.setObjectName(name)
            button.setToolTip(tip)
            button.clicked.connect(callback)
            layout.addWidget(button)

    def _toggle_maximized(self) -> None:
        self._window.showNormal() if self._window.isMaximized() else self._window.showMaximized()

    def mousePressEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_origin = event.globalPosition().toPoint() - self._window.pos()

    def mouseMoveEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if self._drag_origin is not None and event.buttons() & Qt.MouseButton.LeftButton:
            if self._window.isMaximized():
                self._window.showNormal()
            self._window.move(event.globalPosition().toPoint() - self._drag_origin)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        self._drag_origin = None

    def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton:
            self._toggle_maximized()
