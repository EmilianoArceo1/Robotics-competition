from __future__ import annotations

from math import hypot

from ..Coordinator import Coordinator
from ..models import (
    CoordinationContext, CoordinationDecision, CoordinationMessage,
    CoordinationMode, GoalAssignment,
)


class LocalAuctionCoordinator(Coordinator):
    def __init__(self, *, reservation_ttl: float = 2.0) -> None:
        if reservation_ttl <= 0.0:
            raise ValueError("reservation_ttl debe ser positivo")
        self.reservation_ttl = float(reservation_ttl)

    @property
    def mode(self) -> CoordinationMode:
        return CoordinationMode.DECENTRALIZED

    def coordinate(self, context: CoordinationContext) -> CoordinationDecision:
        robot = next(
            (item for item in context.robots if item.robot_id == context.local_robot_id),
            None,
        )
        if robot is None or not robot.available:
            return CoordinationDecision(self.mode, (), (), "local robot unavailable")

        claims: dict[tuple[float, float], CoordinationMessage] = {}
        for message in context.incoming_messages:
            if message.kind != "goal_claim" or message.goal is None:
                continue
            if message.expires_at is not None and message.expires_at < context.timestamp:
                continue
            incumbent = claims.get(message.goal)
            rank = (message.bid if message.bid is not None else float("inf"), message.sender_id)
            incumbent_rank = (
                incumbent.bid if incumbent and incumbent.bid is not None else float("inf"),
                incumbent.sender_id if incumbent else "",
            )
            if incumbent is None or rank < incumbent_rank:
                claims[message.goal] = message

        ranked = sorted(
            (
                (hypot(goal[0] - robot.position[0], goal[1] - robot.position[1]), goal)
                for goal in context.candidate_goals
            ),
            key=lambda item: (item[0], item[1][1], item[1][0]),
        )
        for bid, goal in ranked:
            incumbent = claims.get(goal)
            incumbent_rank = (
                incumbent.bid if incumbent and incumbent.bid is not None else float("inf"),
                incumbent.sender_id if incumbent else "",
            )
            if incumbent is not None and incumbent_rank <= (bid, robot.robot_id):
                continue
            claim = CoordinationMessage(
                robot.robot_id, "goal_claim", goal, bid,
                expires_at=context.timestamp + self.reservation_ttl,
            )
            return CoordinationDecision(
                self.mode,
                (GoalAssignment(robot.robot_id, goal, bid),),
                (claim,),
                "local auction claim",
            )
        return CoordinationDecision(self.mode, (), (), "all goals claimed by peers")


__all__ = ["LocalAuctionCoordinator"]
