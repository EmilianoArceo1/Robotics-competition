from __future__ import annotations

import unittest

from Logic.Exploration import ExplorationManager, ExplorationState


class ExplorationManagerTests(unittest.TestCase):
    def test_selects_candidates_in_strategy_order(self) -> None:
        manager = ExplorationManager()
        decision = manager.select_goal(((3.0, 0.0), (1.0, 0.0)))
        self.assertEqual(decision.goal, (3.0, 0.0))
        self.assertEqual(decision.state, ExplorationState.EXPLORING)

    def test_reached_goal_is_not_selected_again(self) -> None:
        manager = ExplorationManager()
        manager.select_goal(((1.0, 0.0), (2.0, 0.0)))
        manager.mark_reached()
        decision = manager.select_goal(((1.0, 0.0), (2.0, 0.0)))
        self.assertEqual(decision.goal, (2.0, 0.0))
        self.assertEqual(manager.snapshot.visited_goals, 1)

    def test_failed_goal_returns_after_cooldown(self) -> None:
        manager = ExplorationManager(failed_goal_cooldown=2)
        manager.select_goal(((1.0, 0.0),))
        manager.mark_failed()
        self.assertIsNone(manager.select_goal(((1.0, 0.0),)).goal)
        manager.begin_tick()
        manager.begin_tick()
        self.assertEqual(
            manager.select_goal(((1.0, 0.0),)).goal,
            (1.0, 0.0),
        )

    def test_no_candidates_marks_exploration_complete(self) -> None:
        manager = ExplorationManager()
        decision = manager.select_goal(())
        self.assertEqual(decision.state, ExplorationState.COMPLETE)
        self.assertEqual(manager.snapshot.state, ExplorationState.COMPLETE)


if __name__ == "__main__":
    unittest.main()
