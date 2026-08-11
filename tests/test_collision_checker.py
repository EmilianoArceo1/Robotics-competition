from __future__ import annotations

import unittest

from Logic.Robot.CollisionChecker import (
    CollisionChecker,
    distance_segment_to_rect,
)


class CollisionCheckerTests(unittest.TestCase):
    def test_swept_motion_cannot_cross_obstacle(self) -> None:
        checker = CollisionChecker(
            ((2.0, 0.0),), robot_radius=0.3, safety_radius=0.2
        )
        self.assertFalse(checker.collides((0.0, 0.0)))
        self.assertTrue(checker.motion_collides((0.0, 0.0), (3.0, 0.0)))

    def test_safety_radius_updates_clearance(self) -> None:
        checker = CollisionChecker(((2.0, 0.0),), robot_radius=0.3)
        self.assertFalse(checker.collides((1.1, 0.0)))
        checker.configure_safety_radius(0.2)
        self.assertTrue(checker.collides((1.1, 0.0)))

    def test_rounded_corner_does_not_use_square_false_positive(self) -> None:
        checker = CollisionChecker(
            ((0.5, 0.5),), robot_radius=0.35, safety_radius=0.0
        )
        self.assertFalse(checker.check_position((1.3, 1.3)).collision)
        self.assertTrue(checker.check_position((1.2, 1.2)).collision)

    def test_collision_report_explains_blocked_motion(self) -> None:
        checker = CollisionChecker(
            ((0.5, 0.5),), robot_radius=0.2, safety_radius=0.1
        )
        report = checker.check_motion((-1.0, 0.5), (2.0, 0.5))
        self.assertTrue(report.collision)
        self.assertEqual(report.reason, "motion intersects safety envelope")
        self.assertEqual(report.distance, 0.0)
        self.assertIsNotNone(report.obstacle)

    def test_segment_distance_preserves_real_corner_clearance(self) -> None:
        distance = distance_segment_to_rect(
            (1.3, 1.3), (1.5, 1.5), (0.0, 0.0, 1.0, 1.0)
        )
        self.assertGreater(distance, 0.35)


if __name__ == "__main__":
    unittest.main()
