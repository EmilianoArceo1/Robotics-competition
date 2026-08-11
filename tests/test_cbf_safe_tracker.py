from __future__ import annotations

import unittest

from Controllers.safe_tracker_controller import SafeTrackerController
from Logic.Methods.SafeTracking import CBFSafeTracker
from Logic.Robot.BeliefMap import BeliefMap
from Logic.Robot.Physic import RobotState
from Logic.Robot.Sensor import SensorScan


def scan_with_obstacle(x: float, y: float) -> SensorScan:
    return SensorScan([[[x, y], 1]], ())


class CBFSafeTrackerTests(unittest.TestCase):
    def test_nominal_control_passes_without_obstacles(self) -> None:
        tracker = CBFSafeTracker()
        control = tracker.filter_control(
            RobotState(), (1.0, 0.4), BeliefMap(), SensorScan([], ()), 0.05
        )
        self.assertEqual(control, (1.0, 0.4))
        self.assertFalse(tracker.status.active)

    def test_front_obstacle_reduces_linear_acceleration(self) -> None:
        tracker = CBFSafeTracker()
        state = RobotState(x=0.0, y=0.0, theta=0.0, linear_velocity=0.5)
        safe = tracker.filter_control(
            state, (1.0, 0.25), BeliefMap(), scan_with_obstacle(1.5, 0.0), 0.05
        )
        self.assertLess(safe[0], 1.0)
        self.assertEqual(safe[1], 0.25)
        self.assertTrue(tracker.status.active)
        self.assertFalse(tracker.status.emergency)

    def test_violated_envelope_triggers_emergency_braking(self) -> None:
        tracker = CBFSafeTracker()
        safe = tracker.filter_control(
            RobotState(linear_velocity=1.0),
            (1.0, 0.8),
            BeliefMap(),
            scan_with_obstacle(0.8, 0.0),
            0.05,
        )
        self.assertEqual(safe, (-2.0, 0.0))
        self.assertTrue(tracker.status.emergency)

    def test_controller_exposes_cbf_option(self) -> None:
        controller = SafeTrackerController()
        controller.select("HOCBF (obstáculos estáticos)")
        tracker = controller.create(0.7)
        self.assertIsInstance(tracker, CBFSafeTracker)
        self.assertEqual(tracker.safety_radius, 0.7)


if __name__ == "__main__":
    unittest.main()
