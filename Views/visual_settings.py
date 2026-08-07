"""Diálogo persistente para la identidad visual del simulador."""

from collections.abc import Callable

from PySide6.QtCore import QSettings
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QColorDialog, QComboBox, QDialog, QFormLayout, QPushButton, QVBoxLayout

from Controllers.belief_map_controller import BeliefMapController


class VisualSettingsDialog(QDialog):
    def __init__(self, belief: BeliefMapController, settings: QSettings, apply_logo: Callable[[str], None], parent=None) -> None:
        super().__init__(parent)
        self.belief, self.settings, self.apply_logo = belief, settings, apply_logo
        self.setWindowTitle("Apariencia")
        self.setMinimumWidth(360)
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.logo = QComboBox()
        self.logo.addItems(("Círculo", "Dron", "Robot con ruedas"))
        self.logo.setCurrentText(str(settings.value("visual/robot", "Círculo")))
        self.logo.currentTextChanged.connect(self._logo_changed)
        form.addRow("Representación del robot", self.logo)
        for occupancy, label in ((-1, "Desconocido"), (0, "Libre"), (1, "Ocupado")):
            button = QPushButton(self.belief.color_for(occupancy))
            button.clicked.connect(lambda _=False, value=occupancy, target=button: self._color(value, target))
            form.addRow(label, button)
        layout.addLayout(form)
        close = QPushButton("LISTO")
        close.clicked.connect(self.accept)
        layout.addWidget(close)

    def _logo_changed(self, value: str) -> None:
        self.settings.setValue("visual/robot", value)
        self.apply_logo(value)

    def _color(self, occupancy: int, button: QPushButton) -> None:
        color = QColorDialog.getColor(QColor(self.belief.color_for(occupancy)), self)
        if not color.isValid():
            return
        self.belief.set_color(occupancy, color.name())
        self.settings.setValue(f"visual/belief_{occupancy}", color.name())
        button.setText(color.name())
        self.parent().map_view.redraw()
