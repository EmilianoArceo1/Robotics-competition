"""Selección de filtros de seguimiento seguro."""

from collections.abc import Callable

from Logic.Methods.SafeTracking import CBFSafeTracker, NoSafety, SafeTracker

TrackerFactory = Callable[[float], SafeTracker]


class SafeTrackerController:
    def __init__(self) -> None:
        self._methods: dict[str, TrackerFactory] = {
            "Sin filtro": lambda _radius: NoSafety(),
            "HOCBF (obstáculos estáticos)": (
                lambda radius: CBFSafeTracker(safety_radius=radius)
            ),
        }
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

    def create(self, safety_radius: float = 0.0) -> SafeTracker:
        return self._methods[self._selected](safety_radius)

    def register(self, name: str, factory: TrackerFactory) -> None:
        self._methods[name.strip()] = factory
