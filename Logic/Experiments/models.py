"""Entidades inmutables de experimentos reproducibles."""

from __future__ import annotations

from dataclasses import dataclass

Coordinate = tuple[float, float]


@dataclass(frozen=True, slots=True)
class AlgorithmConfiguration:
    path_planner: str
    objective_assigner: str
    clustering: str
    safe_tracker: str
    coordination: str = "Sin coordinación"


@dataclass(frozen=True, slots=True)
class SensorConfiguration:
    sensor_type: str
    field_of_view: float
    detection_radius: float
    grid_size: float


@dataclass(frozen=True, slots=True)
class RobotConfiguration:
    length: float
    width: float
    safety_radius: float
    robot_count: int = 1


@dataclass(frozen=True, slots=True)
class MapConfiguration:
    obstacles: tuple[Coordinate, ...]
    obstacle_size: float
    robot_start_world: Coordinate


@dataclass(frozen=True, slots=True)
class ExperimentConfiguration:
    name: str
    seed: int
    algorithms: AlgorithmConfiguration
    sensor: SensorConfiguration
    robot: RobotConfiguration
    map: MapConfiguration
    schema_version: int = 1


@dataclass(frozen=True, slots=True)
class ExperimentSample:
    elapsed_time: float
    x: float
    y: float
    theta: float
    coverage: float
    navigation_state: str
    exploration_state: str
    current_goal: Coordinate | None
    safety_active: bool
    collision_rejected: bool


@dataclass(frozen=True, slots=True)
class ExperimentSummary:
    outcome: str
    reason: str
    elapsed_time: float
    distance_traveled: float
    coverage: float
    known_cells: int
    total_cells: int
    goals_reached: int
    failed_goals: int
    replans: int
    safety_interventions: int
    rejected_collisions: int


@dataclass(frozen=True, slots=True)
class ExperimentResult:
    configuration: ExperimentConfiguration
    summary: ExperimentSummary
    trajectory: tuple[ExperimentSample, ...]


__all__ = [
    "AlgorithmConfiguration",
    "ExperimentConfiguration",
    "ExperimentResult",
    "ExperimentSample",
    "ExperimentSummary",
    "MapConfiguration",
    "RobotConfiguration",
    "SensorConfiguration",
]
