from __future__ import annotations

import unittest

from Logic.Exploration import (
    ExplorationMetrics,
    ExplorationOutcome,
    ExplorationSnapshot,
    ExplorationState,
)
from Logic.Navigation import NavigationSnapshot, NavigationState


def exploration(state=ExplorationState.EXPLORING, visited=0, failed=0):
    return ExplorationSnapshot(state, None, visited, failed, "test")


def navigation(state=NavigationState.FOLLOWING, replans=0):
    return NavigationSnapshot(state, "test", replans, 0, 0, None, None)


class ExplorationMetricsTests(unittest.TestCase):
    def test_accumulates_coverage_distance_and_events(self) -> None:
        metrics = ExplorationMetrics()
        belief = (((0.0, 0.0), 0), ((1.0, 0.0), 1))
        metrics.record_tick(
            dt=0.5, position=(0.0, 0.0), belief_matrix=belief,
            total_cells=4, exploration=exploration(visited=1),
            navigation=navigation(replans=2), safety_active=True,
            collision_rejected=True,
        )
        metrics.record_tick(
            dt=0.5, position=(3.0, 4.0), belief_matrix=belief,
            total_cells=4, exploration=exploration(visited=1),
            navigation=navigation(replans=2), safety_active=True,
            collision_rejected=False,
        )
        snapshot = metrics.snapshot
        self.assertEqual(snapshot.coverage, 0.5)
        self.assertEqual(snapshot.distance_traveled, 5.0)
        self.assertEqual(snapshot.safety_interventions, 1)
        self.assertEqual(snapshot.rejected_collisions, 1)
        self.assertEqual(snapshot.replans, 2)

    def test_complete_and_partial_outcomes_are_distinct(self) -> None:
        for known, expected in (
            (4, ExplorationOutcome.COMPLETED),
            (2, ExplorationOutcome.PARTIALLY_COMPLETED),
        ):
            metrics = ExplorationMetrics(completion_threshold=0.75)
            belief = [((float(index), 0.0), 0) for index in range(known)]
            metrics.record_tick(
                dt=0.1, position=(0.0, 0.0), belief_matrix=belief,
                total_cells=4, exploration=exploration(ExplorationState.COMPLETE),
                navigation=navigation(NavigationState.EXHAUSTED),
                safety_active=False, collision_rejected=False,
            )
            self.assertEqual(metrics.snapshot.outcome, expected)

    def test_unreachable_frontiers_require_sustained_stall(self) -> None:
        metrics = ExplorationMetrics(stalled_tick_limit=2)
        for _ in range(2):
            metrics.record_tick(
                dt=0.1, position=(0.0, 0.0), belief_matrix=(),
                total_cells=4, exploration=exploration(failed=1),
                navigation=navigation(NavigationState.EXHAUSTED),
                safety_active=False, collision_rejected=False,
            )
        self.assertEqual(
            metrics.snapshot.outcome,
            ExplorationOutcome.NO_REACHABLE_FRONTIERS,
        )


if __name__ == "__main__":
    unittest.main()
