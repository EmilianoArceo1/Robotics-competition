"""Compara algoritmos de retorno manteniendo fija la policy de exploración."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from statistics import mean

import numpy as np

from Logic.Competition import CompetitionConfig, CompetitionWorld
from Logic.Competition.advanced_policies import make_advanced_policy
from Logic.Competition.return_policies import make_return_policy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policies", nargs="+",
                        default=["periodic", "deadline", "payload_adaptive", "link_aware"])
    parser.add_argument("--environments", nargs="+", default=["env1", "env3", "env6"])
    parser.add_argument("--robots", type=int, nargs="+", default=[3, 5])
    parser.add_argument("--max-steps", type=int, default=240)
    parser.add_argument("--relay-period", type=int, default=60)
    parser.add_argument("--num-laser", type=int, default=32)
    parser.add_argument("--output", type=Path, default=Path("experiments/return_policy_comparison"))
    args = parser.parse_args()
    maps = Path(__file__).resolve().parent/"Assets"/"competition_maps"
    rows = []
    for name in args.policies:
        for environment in args.environments:
            for robots in args.robots:
                config = CompetitionConfig(num_robots=robots, start_pose=(15, 15),
                    max_steps=args.max_steps, num_laser=args.num_laser,
                    relay_period=args.relay_period)
                world = CompetitionWorld(maps/environment/"occ_map.npy", config,
                    make_advanced_policy("trajectory_diversified"), make_return_policy(name))
                while world.timestep < config.max_steps:
                    world.step()
                p = config.pd_size
                live = world.live_observation_map()[p:-p, p:-p]
                unreported = sum(int(np.count_nonzero(r.unreported_mask)) for r in world.robots)
                returned = sum(float(np.linalg.norm(r.pose-world.base_pose)) <= 3 for r in world.robots)
                rows.append({"ReturnPolicy": name, "Environment": environment,
                    "Robots": robots, "ReportedCoverage": 100*world.coverage,
                    "LiveCoverage": 100*float(np.mean(live != .5)),
                    "UnreportedCells": unreported, "RobotsAtBase": returned})
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output/"raw.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    summary = []
    for name in args.policies:
        selected = [r for r in rows if r["ReturnPolicy"] == name]
        summary.append({"ReturnPolicy": name,
            "MeanReportedCoverage": mean(r["ReportedCoverage"] for r in selected),
            "WorstReportedCoverage": min(r["ReportedCoverage"] for r in selected),
            "MeanLiveCoverage": mean(r["LiveCoverage"] for r in selected),
            "MeanUnreportedCells": mean(r["UnreportedCells"] for r in selected),
            "MeanRobotsAtBase": mean(r["RobotsAtBase"] for r in selected),
            "Trials": len(selected)})
    summary.sort(key=lambda r: (-r["MeanReportedCoverage"], r["MeanUnreportedCells"]))
    with (args.output/"summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0]); writer.writeheader(); writer.writerows(summary)


if __name__ == "__main__":
    main()
