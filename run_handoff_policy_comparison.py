"""Compara handoffs con exploración y retorno fijos."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from statistics import mean

import numpy as np

from Logic.Competition import CompetitionConfig, CompetitionWorld
from Logic.Competition.advanced_policies import make_advanced_policy
from Logic.Competition.handoff_policies import make_handoff_policy
from Logic.Competition.return_policies import make_return_policy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policies", nargs="+",
                        default=["closest_progress", "payload_progress"])
    parser.add_argument("--environments", nargs="+", default=["env1", "env3"])
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--relay-period", type=int, default=25)
    parser.add_argument("--num-laser", type=int, default=24)
    parser.add_argument("--output", type=Path,
                        default=Path("experiments/handoff_policy_comparison"))
    args = parser.parse_args()
    maps = Path(__file__).resolve().parent/"Assets"/"competition_maps"
    rows = []
    for name in args.policies:
        for environment in args.environments:
            config = CompetitionConfig(num_robots=args.robots, start_pose=(15, 15),
                max_steps=args.max_steps, num_laser=args.num_laser,
                relay_period=args.relay_period)
            world = CompetitionWorld(maps/environment/"occ_map.npy", config,
                make_advanced_policy("trajectory_diversified"),
                make_return_policy("efficient_periodic"), make_handoff_policy(name))
            while world.timestep < config.max_steps:
                world.step()
            unreported = sum(int(np.count_nonzero(r.unreported_mask)) for r in world.robots)
            returned = sum(float(np.linalg.norm(r.pose-world.base_pose)) <= 3
                           for r in world.robots)
            rows.append({"HandoffPolicy": name, "Environment": environment,
                "ReportedCoverage": 100*world.coverage,
                "UnreportedCells": unreported, "RobotsAtBase": returned,
                "Handoffs": world.handoff_count})
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output/"raw.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    summary = []
    for name in args.policies:
        selected = [row for row in rows if row["HandoffPolicy"] == name]
        summary.append({"HandoffPolicy": name,
            "MeanReportedCoverage": mean(row["ReportedCoverage"] for row in selected),
            "WorstReportedCoverage": min(row["ReportedCoverage"] for row in selected),
            "MeanUnreportedCells": mean(row["UnreportedCells"] for row in selected),
            "MeanRobotsAtBase": mean(row["RobotsAtBase"] for row in selected),
            "MeanHandoffs": mean(row["Handoffs"] for row in selected),
            "Trials": len(selected)})
    summary.sort(key=lambda row: (-row["MeanReportedCoverage"], row["MeanUnreportedCells"]))
    with (args.output/"summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0]); writer.writeheader(); writer.writerows(summary)


if __name__ == "__main__":
    main()
