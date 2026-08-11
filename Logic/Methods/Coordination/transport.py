from __future__ import annotations

from typing import Protocol

from .models import CoordinationMessage


class CoordinationTransport(Protocol):
    def publish(self, message: CoordinationMessage) -> None: ...
    def receive(self, robot_id: str, timestamp: float) -> tuple[CoordinationMessage, ...]: ...


__all__ = ["CoordinationTransport"]
