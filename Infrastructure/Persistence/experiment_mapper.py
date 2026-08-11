"""Mapeo entre entidades de dominio y documentos serializables."""

from __future__ import annotations

from Logic.Experiments import (
    AlgorithmConfiguration,
    ExperimentConfiguration,
    ExperimentResult,
    ExperimentSample,
    ExperimentSummary,
    MapConfiguration,
    RobotConfiguration,
    SensorConfiguration,
)


def result_to_document(result: ExperimentResult) -> dict[str, object]:
    configuration = result.configuration
    return {
        "schema_version": configuration.schema_version,
        "configuration": {
            "name": configuration.name,
            "seed": configuration.seed,
            "algorithms": {
                "path_planner": configuration.algorithms.path_planner,
                "objective_assigner": configuration.algorithms.objective_assigner,
                "clustering": configuration.algorithms.clustering,
                "safe_tracker": configuration.algorithms.safe_tracker,
                "coordination": configuration.algorithms.coordination,
            },
            "sensor": {
                "sensor_type": configuration.sensor.sensor_type,
                "field_of_view": configuration.sensor.field_of_view,
                "detection_radius": configuration.sensor.detection_radius,
                "grid_size": configuration.sensor.grid_size,
            },
            "robot": {
                "length": configuration.robot.length,
                "width": configuration.robot.width,
                "safety_radius": configuration.robot.safety_radius,
                "robot_count": configuration.robot.robot_count,
            },
            "map": {
                "obstacles": [list(point) for point in configuration.map.obstacles],
                "obstacle_size": configuration.map.obstacle_size,
                "robot_start_world": list(configuration.map.robot_start_world),
            },
        },
        "summary": {
            field: getattr(result.summary, field)
            for field in ExperimentSummary.__dataclass_fields__
        },
        "trajectory": [
            {
                "elapsed_time": sample.elapsed_time,
                "x": sample.x,
                "y": sample.y,
                "theta": sample.theta,
                "coverage": sample.coverage,
                "navigation_state": sample.navigation_state,
                "exploration_state": sample.exploration_state,
                "current_goal": (
                    list(sample.current_goal)
                    if sample.current_goal is not None else None
                ),
                "safety_active": sample.safety_active,
                "collision_rejected": sample.collision_rejected,
            }
            for sample in result.trajectory
        ],
    }


def document_to_result(document: dict[str, object]) -> ExperimentResult:
    config = document["configuration"]
    summary = document["summary"]
    trajectory = document["trajectory"]
    if not isinstance(config, dict) or not isinstance(summary, dict):
        raise ValueError("Documento de experimento inválido")
    algorithms, sensor, robot, map_config = (
        config["algorithms"], config["sensor"], config["robot"], config["map"]
    )
    if not all(isinstance(value, dict) for value in (algorithms, sensor, robot, map_config)):
        raise ValueError("Configuración de experimento inválida")
    configuration = ExperimentConfiguration(
        str(config["name"]),
        int(config["seed"]),
        AlgorithmConfiguration(**algorithms),
        SensorConfiguration(**sensor),
        RobotConfiguration(**robot),
        MapConfiguration(
            tuple((float(p[0]), float(p[1])) for p in map_config["obstacles"]),
            float(map_config["obstacle_size"]),
            tuple(float(v) for v in map_config["robot_start_world"]),
        ),
        int(document.get("schema_version", 1)),
    )
    samples = []
    if not isinstance(trajectory, list):
        raise ValueError("La trayectoria debe ser una lista")
    for raw in trajectory:
        goal = raw.get("current_goal")
        samples.append(
            ExperimentSample(
                float(raw["elapsed_time"]), float(raw["x"]), float(raw["y"]),
                float(raw["theta"]), float(raw["coverage"]),
                str(raw["navigation_state"]), str(raw["exploration_state"]),
                None if goal is None else (float(goal[0]), float(goal[1])),
                bool(raw["safety_active"]), bool(raw["collision_rejected"]),
            )
        )
    return ExperimentResult(
        configuration,
        ExperimentSummary(**summary),
        tuple(samples),
    )
