from __future__ import annotations

from abc import ABC, abstractmethod

from .models import CoordinationContext, CoordinationDecision, CoordinationMode


class Coordinator(ABC):
    @property
    @abstractmethod
    def mode(self) -> CoordinationMode:
        raise NotImplementedError

    @abstractmethod
    def coordinate(self, context: CoordinationContext) -> CoordinationDecision:
        raise NotImplementedError


__all__ = ["Coordinator"]
