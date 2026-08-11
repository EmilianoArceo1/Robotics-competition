from __future__ import annotations

import unittest

from Logic.Navigation import NavigationState, NavigationSupervisor


class NavigationSupervisorTests(unittest.TestCase):
    def test_route_lifecycle(self) -> None:
        supervisor = NavigationSupervisor()

        supervisor.planning()
        self.assertEqual(supervisor.snapshot.state, NavigationState.PLANNING)

        supervisor.route_accepted((2.0, 3.0))
        self.assertEqual(supervisor.snapshot.state, NavigationState.FOLLOWING)
        self.assertEqual(supervisor.snapshot.current_goal, (2.0, 3.0))

        supervisor.goal_reached()
        self.assertEqual(supervisor.snapshot.state, NavigationState.GOAL_REACHED)
        self.assertIsNone(supervisor.snapshot.current_goal)

    def test_failed_goal_is_temporarily_rejected(self) -> None:
        supervisor = NavigationSupervisor(failed_goal_cooldown=2)
        goal = (4.0, -1.0)

        supervisor.planning_failed(goal, "unreachable")
        self.assertFalse(supervisor.goal_available(goal))
        self.assertEqual(supervisor.snapshot.failed_goal_count, 1)
        self.assertEqual(supervisor.snapshot.state, NavigationState.RECOVERING)

        supervisor.begin_tick()
        self.assertFalse(supervisor.goal_available(goal))
        supervisor.begin_tick()
        self.assertTrue(supervisor.goal_available(goal))

    def test_repeated_safety_stop_marks_robot_as_blocked(self) -> None:
        supervisor = NavigationSupervisor(blocked_tick_limit=3)
        supervisor.route_accepted((5.0, 0.0))

        self.assertFalse(
            supervisor.observe_motion(
                (0.0, 0.0), commanded=True,
                safety_active=True, collision=False,
            )
        )
        for _ in range(2):
            self.assertFalse(
                supervisor.observe_motion(
                    (0.0, 0.0), commanded=True,
                    safety_active=True, collision=False,
                )
            )
        self.assertTrue(
            supervisor.observe_motion(
                (0.0, 0.0), commanded=True,
                safety_active=True, collision=False,
            )
        )
        self.assertEqual(supervisor.snapshot.state, NavigationState.BLOCKED)

    def test_motion_progress_clears_block_counter(self) -> None:
        supervisor = NavigationSupervisor(blocked_tick_limit=3)
        supervisor.observe_motion(
            (0.0, 0.0), commanded=True,
            safety_active=True, collision=False,
        )
        supervisor.observe_motion(
            (0.0, 0.0), commanded=True,
            safety_active=True, collision=False,
        )
        supervisor.observe_motion(
            (0.1, 0.0), commanded=True,
            safety_active=True, collision=False,
        )

        self.assertEqual(supervisor.snapshot.blocked_ticks, 0)


if __name__ == "__main__":
    unittest.main()
