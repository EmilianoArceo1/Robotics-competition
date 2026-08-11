"""CLI para generar raw.csv, summary.csv y leave_one_out.csv."""
import argparse
from pathlib import Path

from Logic.Competition.experiments import (CompetitionSweepRunner, export_csv,
    leave_one_environment_out, simplex_weights, summarize)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=float, default=0.25)
    parser.add_argument("--robots", type=int, nargs="+", default=[1,2,3,4,5])
    parser.add_argument("--starts", type=int, nargs=2, action="append", default=None,
                        metavar=("X", "Y"))
    parser.add_argument("--environments", nargs="+", default=[f"env{i}" for i in range(1,8)])
    parser.add_argument("--num-laser", type=int, default=2500)
    parser.add_argument("--policy", choices=("nearest", "weighted", "nbv",
                        "adaptive", "tuned", "gain_per_cost", "coordinated",
                        "hybrid_005", "hybrid_010", "hybrid_020",
                        "intent_nearest", "soft_intent_nearest",
                        "regret_25", "regret_50", "regret_75",
                        "intent_only", "trajectory_only", "intent_tight",
                        "trajectory_050", "trajectory_075", "trajectory_125",
                        "trajectory_diversified", "recent_trail",
                        "voronoi_nearest", "frontier_reservation",
                        "elastic_trajectory", "clearance_utility",
                        "detour_capped"), default="nbv")
    parser.add_argument("--max-steps", type=int, default=None,
                        help="Límite proxy; omitir para budgets oficiales")
    parser.add_argument("--nbv-rays", type=int, default=180)
    parser.add_argument("--nbv-candidates", type=int, default=160)
    parser.add_argument("--output", type=Path, default=Path("experiments/competition_sweep"))
    args = parser.parse_args()
    starts = [tuple(value) for value in (args.starts or [[15,15]])]
    root = Path(__file__).resolve().parent / "Assets" / "competition_maps"
    results = CompetitionSweepRunner(root).run(simplex_weights(args.step),
        args.environments, args.robots, starts, num_laser=args.num_laser,
        policy_name=args.policy, max_steps_override=args.max_steps,
        nbv_rays=args.nbv_rays, nbv_candidates=args.nbv_candidates)
    export_csv(results, args.output/"raw.csv")
    export_csv(summarize(results), args.output/"summary.csv")
    export_csv(leave_one_environment_out(results), args.output/"leave_one_out.csv")


if __name__ == "__main__":
    main()
