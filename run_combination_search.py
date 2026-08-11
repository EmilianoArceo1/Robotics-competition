"""Búsqueda conjunta de decide + should_relay + decide_relay_handoff."""

from __future__ import annotations

import argparse
import csv
from itertools import product
from pathlib import Path

import numpy as np

from Logic.Competition import CompetitionConfig, CompetitionWorld
from Logic.Competition.advanced_policies import make_advanced_policy
from Logic.Competition.handoff_policies import make_handoff_policy
from Logic.Competition.return_policies import make_return_policy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exploration", nargs="+", default=[
        "trajectory_diversified", "detour_capped", "voronoi_nearest"])
    parser.add_argument("--return-policies", nargs="+", default=[
        "selective_courier", "efficient_periodic", "periodic"])
    parser.add_argument("--handoff", nargs="+", default=[
        "payload_progress", "returning_courier", "link_quality"])
    parser.add_argument("--environments", nargs="+", default=["env1"])
    parser.add_argument("--robots", type=int, nargs="+", default=[5])
    parser.add_argument("--max-steps", type=int, default=40)
    parser.add_argument("--relay-period", type=int, default=15)
    parser.add_argument("--num-laser", type=int, default=8)
    parser.add_argument("--output", type=Path,
                        default=Path("experiments/combination_search"))
    args = parser.parse_args()
    maps = Path(__file__).resolve().parent/"Assets"/"competition_maps"
    rows = []
    combinations = product(args.exploration, args.return_policies, args.handoff,
                           args.environments, args.robots)
    for exploration, returning, handoff, environment, robots in combinations:
        config = CompetitionConfig(num_robots=robots, start_pose=(15, 15),
            max_steps=args.max_steps, num_laser=args.num_laser,
            relay_period=args.relay_period)
        world = CompetitionWorld(maps/environment/"occ_map.npy", config,
            make_advanced_policy(exploration), make_return_policy(returning),
            make_handoff_policy(handoff))
        while world.timestep < config.max_steps:
            world.step()
        returned = sum(float(np.linalg.norm(r.pose-world.base_pose)) <= 3
                       for r in world.robots)
        pending = sum(int(np.count_nonzero(r.unreported_mask)) for r in world.robots)
        coverage = 100*world.coverage
        return_fraction = returned/robots
        rows.append({"Exploration": exploration, "Return": returning,
            "Handoff": handoff, "Environment": environment, "Robots": robots,
            "ReportedCoverage": coverage, "RobotsAtBase": returned,
            "UnreportedCells": pending, "Handoffs": world.handoff_count,
            "SafeScore": coverage*return_fraction})
    rows.sort(key=lambda row: (-row["SafeScore"], -row["ReportedCoverage"],
                               row["UnreportedCells"], row["Handoffs"]))
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output/"ranking.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)


if __name__ == "__main__":
    main()
