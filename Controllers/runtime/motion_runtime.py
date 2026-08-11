"""Fase de control, integración física y rechazo de colisiones."""

from Controllers.map_controller import MapController
from Controllers.robot_control_controller import RobotControlController
from Logic.Robot.CollisionChecker import CollisionChecker, CollisionReport
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
    ) -> CollisionReport:
        completed = control.apply_tracking_control(track, dt)
        if completed:
            return CollisionReport(False, "route complete")

        state = control.physics.state
        previous_pose = state.x, state.y, state.theta
        state = control.physics.step(dt)
        report = (
            collision_checker.check_motion(previous_pose[:2], (state.x, state.y))
            if collision_checker is not None
            else CollisionReport(False)
        )
        if report.collision:
            state.x, state.y, state.theta = previous_pose
            control.physics.stop()
        self.map_controller.update_robot_pose(state.x, state.y, state.theta)
        return report
