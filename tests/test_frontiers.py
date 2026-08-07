from __future__ import annotations

import importlib
import unittest

from Logic.Robot.BeliefMap import BeliefMap


Frontiers = importlib.import_module(
    "Logic.Methods.Objective Assign.Frontiers approaches.Frontiers"
).Frontiers


class RawFrontiers(Frontiers):
    def cluster_frontiers(self, frontiers):
        return frontiers


class FrontierTests(unittest.TestCase):
    def test_complete_region_has_only_external_boundary(self) -> None:
        for resolution in (0.1, 0.3, 0.5, 0.7, 1.0, 1.6):
            with self.subTest(resolution=resolution):
                belief = BeliefMap(
                    [
                        [[column * resolution, row * resolution], 0]
                        for row in range(-5, 6)
                        for column in range(-5, 6)
                    ]
                )
                self.assertEqual(
                    len(RawFrontiers(cell_size=resolution).get_frontiers(belief)),
                    40,
                )


if __name__ == "__main__":
    unittest.main()
