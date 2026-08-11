from __future__ import annotations

from Logic.Methods.Coordination import CoordinationMessage


class InMemoryCoordinationTransport:
    """Bus determinista para simulación; no conoce robots ni coordinadores."""

    def __init__(self) -> None:
        self._messages: list[CoordinationMessage] = []

    def publish(self, message: CoordinationMessage) -> None:
        self._messages.append(message)

    def receive(
        self, robot_id: str, timestamp: float
    ) -> tuple[CoordinationMessage, ...]:
        return tuple(
            message
            for message in self._messages
            if message.sender_id != robot_id
            and message.recipient_id in (None, robot_id)
            and (message.expires_at is None or message.expires_at >= timestamp)
        )

    def clear(self) -> None:
        self._messages.clear()


__all__ = ["InMemoryCoordinationTransport"]
