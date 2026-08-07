"""Selección de filtros de seguimiento seguro."""

from Logic.Methods.SafeTracking import NoSafety, SafeTracker


class SafeTrackerController:
    def __init__(self) -> None:
        self._methods: dict[str, type[SafeTracker]] = {"Sin filtro": NoSafety}
        self._selected = "Sin filtro"

    @property
    def available_methods(self) -> tuple[str, ...]:
        return tuple(self._methods)

    @property
    def selected_method(self) -> str:
        return self._selected

    def select(self, name: str) -> None:
        if name not in self._methods:
            raise ValueError(f"Safe tracker no registrado: {name}")
        self._selected = name

    def create(self) -> SafeTracker:
        return self._methods[self._selected]()

    def register(self, name: str, tracker: type[SafeTracker]) -> None:
        if not issubclass(tracker, SafeTracker):
            raise TypeError("tracker debe implementar SafeTracker")
        self._methods[name.strip()] = tracker
