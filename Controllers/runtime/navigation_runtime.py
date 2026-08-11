"""Planificación, validación y recuperación de rutas."""

from __future__ import annotations

from Controllers.map_controller import MapController
from Controllers.robot_control_controller import RobotControlController
from Logic.Navigation import NavigationState, NavigationSupervisor
from Logic.Exploration import ExplorationManager, ExplorationState
from Logic.Planning.path_simplifier import PathSimplifier
from Logic.Robot.Track import Track


class NavigationRuntime:
    def __init__(self, map_controller: MapController) -> None:
        self.map_controller = map_controller
        self.supervisor = NavigationSupervisor()
        self.exploration = ExplorationManager()

    def reset(self) -> None:
        self.supervisor.reset()
        self.exploration.reset()

    def _update_costmap(
        self, control: RobotControlController, track: Track
    ) -> None:
        update_belief = getattr(track, "update_belief_map", None)
        if callable(update_belief):
            update_belief(control.belief_map)

    @staticmethod
    def _route_is_valid(
        control: RobotControlController, track: Track
    ) -> bool:
        costmap = getattr(track, "costmap", None)
        geometry = getattr(track, "geometry", None)
        if costmap is None or geometry is None:
            return True
        remaining = track.waypoints.matrix[track.waypoints.current_index:]
        if not remaining:
            return True
        cells = [
            geometry.world_to_cell(control.physics.state.x, control.physics.state.y),
            *(geometry.world_to_cell(*point) for point in remaining),
        ]
        simplifier = PathSimplifier(costmap)
        return all(
            simplifier.segment_is_safe(start, end)
            for start, end in zip(cells, cells[1:])
        )

    def ensure_route(
        self,
        control: RobotControlController,
        track: Track,
        candidate_goals: tuple[tuple[float, float], ...],
        *,
        coordination_error: str | None = None,
    ) -> None:
        self.supervisor.begin_tick()
        self.exploration.begin_tick()
        self._update_costmap(control, track)

        if self.supervisor.state == NavigationState.BLOCKED:
            if self.supervisor.current_goal is not None:
                self.exploration.mark_failed(self.supervisor.current_goal)
                self.supervisor.planning_failed(
                    self.supervisor.current_goal,
                    "blocked goal temporarily rejected",
                )
            track.waypoints.replace(())

        if not track.route_complete:
            if self._route_is_valid(control, track):
                return
            track.waypoints.replace(())
            self.supervisor.route_invalidated(
                "new observation invalidated active route"
            )
        elif self.supervisor.state == NavigationState.FOLLOWING:
            self.exploration.mark_reached(self.supervisor.current_goal)
            self.supervisor.goal_reached()

        self.supervisor.planning()
        if coordination_error is not None:
            self.supervisor.exhausted(
                f"coordination failed: {coordination_error}"
            )
            return
        candidates = list(candidate_goals)

        state = control.physics.state
        attempted = False
        decision = None
        for _ in range(len(candidates) + 1):
            decision = self.exploration.select_goal(candidates)
            if decision.goal is None:
                break
            goal = decision.goal
            if not self.supervisor.goal_available(goal):
                self.exploration.mark_failed(goal)
                continue
            attempted = True
            try:
                track.create_route((state.x, state.y), goal)
            except Exception as error:
                self.exploration.mark_failed(goal)
                self.supervisor.planning_failed(
                    goal, f"route planning failed: {error}"
                )
                continue
            self.supervisor.route_accepted(goal)
            return
        if attempted:
            last_error = self.supervisor.reason
            self.supervisor.exhausted(
                f"all candidate routes failed; {last_error}"
            )
        elif decision is not None:
            self.supervisor.exhausted(
                "exploration complete"
                if decision.state == ExplorationState.COMPLETE
                else decision.reason
            )
        else:
            self.supervisor.exhausted("no exploration decision")

    def observe_motion(
        self,
        control: RobotControlController,
        track: Track,
        *,
        collision: bool,
    ) -> None:
        tracker_status = getattr(control.safe_tracker, "status", None)
        safety_active = bool(
            tracker_status is not None and tracker_status.active
        )
        blocked = self.supervisor.observe_motion(
            (control.physics.state.x, control.physics.state.y),
            commanded=not track.route_complete,
            safety_active=safety_active,
            collision=collision,
        )
        if blocked:
            track.waypoints.replace(())

    def publish_route(self, track: Track) -> None:
        self.map_controller.update_route(
            tuple(
                (float(point[0]), float(point[1]))
                for point in track.waypoints.matrix
            ),
            track.waypoints.current_index,
        )
