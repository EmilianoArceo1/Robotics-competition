"""Fase de asignación de objetivo y publicación de ruta."""

from Controllers.map_controller import MapController
from Controllers.robot_control_controller import RobotControlController
from Logic.Robot.Track import Track


class NavigationRuntime:
    def __init__(self, map_controller: MapController) -> None:
        self.map_controller = map_controller

    def ensure_route(self, control: RobotControlController, track: Track) -> None:
        if track.route_complete:
            control.create_route_to_assigned_goal(track)

    def publish_route(self, track: Track) -> None:
        self.map_controller.update_route(
            tuple(
                (float(point[0]), float(point[1]))
                for point in track.waypoints.matrix
            ),
            track.waypoints.current_index,
        )
