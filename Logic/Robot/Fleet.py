"""Agregado de robots físicos identificables dentro de una flota."""

from __future__ import annotations

from dataclasses import dataclass

from .Physic import RobotPhysics, RobotState


@dataclass(slots=True)
class FleetMember:
    robot_id: str
    physics: RobotPhysics
    controllable: bool


class RobotFleet:
    def __init__(
        self,
        primary: RobotPhysics,
        count: int = 1,
        *,
        spawn_spacing: float = 0.8,
    ) -> None:
        if not 1 <= int(count) <= 20:
            raise ValueError("La flota debe contener entre 1 y 20 robots")
        if spawn_spacing <= 0.0:
            raise ValueError("spawn_spacing debe ser positivo")
        self.members = [FleetMember("robot-1", primary, True)]
        for index in range(1, int(count)):
            physics = RobotPhysics(
                geometry=primary.geometry,
                limits=primary.limits,
                state=RobotState(y=-index * float(spawn_spacing)),
            )
            self.members.append(
                FleetMember(f"robot-{index + 1}", physics, False)
            )

    @property
    def primary(self) -> FleetMember:
        return self.members[0]

    def __len__(self) -> int:
        return len(self.members)


__all__ = ["FleetMember", "RobotFleet"]
