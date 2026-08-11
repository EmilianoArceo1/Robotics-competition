"""Compara baselines y variantes propuestas bajo un protocolo idéntico."""
import argparse
from pathlib import Path

from Logic.Competition import UtilityWeights
from Logic.Competition.experiments import CompetitionSweepRunner, export_csv, summarize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--environments", nargs="+", default=["env1","env3","env6"])
    parser.add_argument("--robots", type=int, nargs="+", default=[3])
    parser.add_argument("--starts", type=int, nargs=2, action="append", default=None)
    parser.add_argument("--num-laser", type=int, default=64)
    parser.add_argument("--max-steps", type=int, default=50)
    parser.add_argument("--nbv-rays", type=int, default=24)
    parser.add_argument("--nbv-candidates", type=int, default=40)
    parser.add_argument("--output", type=Path, default=Path("experiments/policy_comparison"))
    parser.add_argument("--policies", nargs="+",
                        default=["nearest","nbv","tuned","adaptive",
                                 "gain_per_cost","coordinated"])
    args = parser.parse_args()
    root = Path(__file__).resolve().parent/"Assets"/"competition_maps"
    runner = CompetitionSweepRunner(root)
    starts = [tuple(value) for value in (args.starts or [[15,15]])]
    weights = (UtilityWeights(.5,.25,.2,.05),)
    results = []
    for policy in args.policies:
        results.extend(runner.run(weights, args.environments, args.robots, starts,
            num_laser=args.num_laser, policy_name=policy,
            max_steps_override=args.max_steps, nbv_rays=args.nbv_rays,
            nbv_candidates=args.nbv_candidates))
    export_csv(results, args.output/"raw.csv")
    export_csv(summarize(results), args.output/"summary.csv")


if __name__ == "__main__":
    main()
