"""Fase de sensado y actualización del mapa de creencias."""

from __future__ import annotations

from math import ceil

from Controllers.belief_map_controller import BeliefMapController
from Controllers.map_controller import MapController
from Controllers.robot_control_controller import RobotControlController


class PerceptionRuntime:
    def __init__(
        self,
        map_controller: MapController,
        belief_controller: BeliefMapController | None,
    ) -> None:
        self.map_controller = map_controller
        self.belief_controller = belief_controller

    def run(self, control: RobotControlController, grid_size: float) -> None:
        environment = self.map_controller.simulation_map.occupancy_matrix(
            padding=ceil(control.sensor.detection_radius) + 2,
            cell_size=grid_size,
        )
        control.detect(environment)
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
