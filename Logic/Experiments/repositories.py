"""Puertos de persistencia; el dominio no conoce archivos ni formatos."""

from __future__ import annotations

from typing import Protocol

from .models import ExperimentResult


class ExperimentRepository(Protocol):
    def save(self, result: ExperimentResult, destination: str) -> None: ...
    def load(self, source: str) -> ExperimentResult: ...


class ExperimentExporter(Protocol):
    def export(self, result: ExperimentResult, destination: str) -> None: ...


__all__ = ["ExperimentExporter", "ExperimentRepository"]
