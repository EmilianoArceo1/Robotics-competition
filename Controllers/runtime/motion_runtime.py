"""Fase de control, integración física y rechazo de colisiones."""

from Controllers.map_controller import MapController
from Controllers.robot_control_controller import RobotControlController
from Logic.Robot.CollisionChecker import CollisionChecker
from Logic.Robot.Track import Track


class MotionRuntime:
    def __init__(self, map_controller: MapController) -> None:
        self.map_controller = map_controller

    def run(
        self,
        control: RobotControlController,
        track: Track,
        collision_checker: CollisionChecker | None,
        dt: float,
    ) -> bool:
        completed = control.apply_tracking_control(track, dt)
        if completed:
            return True

        state = control.physics.state
        previous_pose = state.x, state.y, state.theta
        state = control.physics.step(dt)
        if (
            collision_checker is not None
            and collision_checker.motion_collides(previous_pose[:2], (state.x, state.y))
        ):
            state.x, state.y, state.theta = previous_pose
            control.physics.stop()
        self.map_controller.update_robot_pose(state.x, state.y, state.theta)
        return False
