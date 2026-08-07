from __future__ import annotations

import unittest

from Logic.Robot.CollisionChecker import CollisionChecker


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


if __name__ == "__main__":
    unittest.main()
