"""Búsqueda corta alrededor del NBV actual, para iteración de diseño."""
import argparse
from pathlib import Path

from Logic.Competition import UtilityWeights
from Logic.Competition.experiments import CompetitionSweepRunner, export_csv, summarize


CANDIDATES = (
    UtilityWeights(.60,.15,.20,.05),
    UtilityWeights(.55,.15,.25,.05),
    UtilityWeights(.50,.10,.35,.05),
    UtilityWeights(.60,.10,.25,.05),
    UtilityWeights(.50,.20,.25,.05),
    UtilityWeights(.65,.15,.15,.05),
    UtilityWeights(.50,.25,.20,.05),  # control actual
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path,
                        default=Path("experiments/nbv_weight_iteration"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent/"Assets"/"competition_maps"
    results = CompetitionSweepRunner(root).run(
        CANDIDATES, ("env1","env3","env6"), (3,), ((15,15),),
        num_laser=32, policy_name="nbv", max_steps_override=20,
        nbv_rays=16, nbv_candidates=20,
    )
    export_csv(results, args.output/"raw.csv")
    export_csv(summarize(results), args.output/"summary.csv")


if __name__ == "__main__":
    main()
