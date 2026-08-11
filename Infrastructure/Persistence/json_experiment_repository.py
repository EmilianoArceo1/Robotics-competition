from __future__ import annotations

import json
from pathlib import Path

from Logic.Experiments import ExperimentResult
from .experiment_mapper import document_to_result, result_to_document


class JsonExperimentRepository:
    def save(self, result: ExperimentResult, destination: str) -> None:
        path = Path(destination)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(result_to_document(result), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def load(self, source: str) -> ExperimentResult:
        document = json.loads(Path(source).read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            raise ValueError("El JSON debe contener un objeto")
        return document_to_result(document)


__all__ = ["JsonExperimentRepository"]
