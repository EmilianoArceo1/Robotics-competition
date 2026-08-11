"""Fase de sensado y actualización del mapa de creencias."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.robot_control_controller import RobotControlController


@dataclass(frozen=True, slots=True)
class PerceptionResult:
    total_cells: int


class PerceptionRuntime:
    def __init__(
        self,
        map_controller: MapController,
        belief_controller: BeliefMapController | None,
    ) -> None:
        self.map_controller = map_controller
        self.belief_controller = belief_controller

    def run(
        self, control: RobotControlController, grid_size: float
    ) -> PerceptionResult:
        environment = self.map_controller.simulation_map.occupancy_matrix(
            padding=ceil(control.sensor.detection_radius) + 2,
            cell_size=grid_size,
        )
        simulation_map = self.map_controller.simulation_map
        control.detect(
            environment,
            occluders=simulation_map.local_obstacles,
            obstacle_size=simulation_map.obstacle_size,
        )
        if self.belief_controller is not None:
            self.belief_controller.update(control.belief_map, environment)
        if control.last_scan is not None:
            self.map_controller.update_sensor_visibility(
                control.last_scan.visibility_polygon
            )
        get_frontiers = getattr(control.objective_assigner, "get_frontiers", None)
        if callable(get_frontiers):
            frontiers = get_frontiers(control.belief_map)
            self.map_controller.update_frontiers(
                tuple((float(point[0]), float(point[1])) for point in frontiers)
            )
            clusters = getattr(
                control.objective_assigner, "frontier_clusters", ()
            )
            self.map_controller.update_frontier_clusters(
                tuple(
                    tuple(
                        (float(point[0]), float(point[1]))
                        for point in cluster.cells
                    )
                    for cluster in clusters
                )
            )
        return PerceptionResult(total_cells=len(environment))
