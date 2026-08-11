from __future__ import annotations

import unittest

from Controllers.coordination_controller import CoordinationController
from Infrastructure.Communication import InMemoryCoordinationTransport
from Logic.Methods.Coordination import (
    CentralizedGreedyCoordinator, CoordinationContext, CoordinationMessage,
    CoordinationMode, LocalAuctionCoordinator, RobotCoordinationState,
)


def robot(robot_id: str, x: float) -> RobotCoordinationState:
    return RobotCoordinationState(robot_id, (x, 0.0), 0.0)


class CoordinationTests(unittest.TestCase):
    def test_centralized_assignments_are_globally_unique(self) -> None:
        decision = CentralizedGreedyCoordinator().coordinate(
            CoordinationContext(
                0.0, "r1", (robot("r1", 0.0), robot("r2", 10.0)),
                ((1.0, 0.0), (9.0, 0.0)),
            )
        )
        self.assertEqual(decision.mode, CoordinationMode.CENTRALIZED)
        self.assertEqual(len(decision.assignments), 2)
        self.assertEqual(
            len({assignment.goal for assignment in decision.assignments}), 2
        )
        self.assertEqual(decision.goal_for("r1"), (1.0, 0.0))
        self.assertEqual(decision.goal_for("r2"), (9.0, 0.0))

    def test_decentralized_auction_respects_better_peer_claim(self) -> None:
        decision = LocalAuctionCoordinator().coordinate(
            CoordinationContext(
                1.0, "r1", (robot("r1", 0.0),),
                ((1.0, 0.0), (2.0, 0.0)),
                (CoordinationMessage("r2", "goal_claim", (1.0, 0.0), 0.5,
                                     expires_at=2.0),),
            )
        )
        self.assertEqual(decision.mode, CoordinationMode.DECENTRALIZED)
        self.assertEqual(decision.goal_for("r1"), (2.0, 0.0))
        self.assertEqual(decision.outgoing_messages[0].kind, "goal_claim")

    def test_expired_claim_does_not_reserve_goal(self) -> None:
        decision = LocalAuctionCoordinator().coordinate(
            CoordinationContext(
                3.0, "r1", (robot("r1", 0.0),), ((1.0, 0.0),),
                (CoordinationMessage("r2", "goal_claim", (1.0, 0.0), 0.1,
                                     expires_at=2.0),),
            )
        )
        self.assertEqual(decision.goal_for("r1"), (1.0, 0.0))

    def test_transport_filters_sender_recipient_and_expiration(self) -> None:
        transport = InMemoryCoordinationTransport()
        transport.publish(CoordinationMessage("r1", "broadcast", expires_at=2.0))
        transport.publish(CoordinationMessage("r2", "private", recipient_id="r3"))
        self.assertEqual(len(transport.receive("r2", 1.0)), 1)
        self.assertEqual(transport.receive("r2", 3.0), ())

    def test_controller_exposes_all_modes(self) -> None:
        controller = CoordinationController()
        for method in (
            "Sin coordinación", "Centralizado greedy", "Subasta descentralizada"
        ):
            controller.select(method)
            self.assertIsNotNone(controller.create())


if __name__ == "__main__":
    unittest.main()
