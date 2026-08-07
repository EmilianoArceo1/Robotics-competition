import sys

from PySide6.QtWidgets import QApplication

from Views.main_view import MainView


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Exploration Lab")
    app.setStyle("Fusion")
    window = MainView()
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
