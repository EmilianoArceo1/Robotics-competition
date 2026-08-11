"""Barridos reproducibles y tablas para tuning de policies."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from itertools import product
from pathlib import Path
from statistics import mean, median, pstdev

from .environment import (CompetitionConfig, CompetitionWorld,
                          NearestFrontierPolicy, UtilityWeights,
                          WeightedUtilityPolicy)
from .next_best_view import NextBestViewPolicy
from .advanced_policies import make_advanced_policy
from .information_gain import PotentialVisibilityInformationGain
from .viewpoints import FrontierOcclusionViewpointGenerator


ENV_SIZE = {"env1":"Small", "env2":"Small", "env3":"Medium",
            "env4":"Medium", "env5":"Medium", "env6":"Large", "env7":"Large"}
MULTI_BUDGET = {"Small":500, "Medium":1000, "Large":1500}
SINGLE_BUDGET = {"Small":1000, "Medium":1500, "Large":2000}


@dataclass(frozen=True, slots=True)
class TrialResult:
    weights: str
    wIG: float
    wC: float
    wR: float
    wL: float
    environment: str
    size: str
    robots: int
    start_x: int
    start_y: int
    max_steps: int
    coverage: float
    policy: str = "weighted"


def simplex_weights(step: float = 0.25) -> tuple[UtilityWeights, ...]:
    divisions = round(1.0 / float(step))
    if divisions <= 0 or not abs(divisions * step - 1.0) < 1e-9:
        raise ValueError("step debe dividir exactamente el intervalo [0, 1]")
    return tuple(UtilityWeights(a/divisions, b/divisions, c/divisions, d/divisions)
                 for a in range(divisions+1) for b in range(divisions+1-a)
                 for c in range(divisions+1-a-b) for d in (divisions-a-b-c,))


class CompetitionSweepRunner:
    def __init__(self, maps_root: str | Path) -> None:
        self.maps_root = Path(maps_root)

    def run(self, weights, environments=tuple(f"env{i}" for i in range(1,8)),
            robot_counts=(1,2,3,4,5), start_poses=((15,15),),
            *, num_laser: int = 2500,
            policy_name: str = "weighted", max_steps_override: int | None = None,
            nbv_rays: int = 180, nbv_candidates: int = 160) -> list[TrialResult]:
        results = []
        for weight, environment, robots, start in product(weights, environments, robot_counts, start_poses):
            size = ENV_SIZE[environment]
            budget = (SINGLE_BUDGET if robots == 1 else MULTI_BUDGET)[size]
            if max_steps_override is not None:
                budget = min(budget, int(max_steps_override))
            config = CompetitionConfig(num_robots=robots, start_pose=tuple(start),
                max_steps=budget, num_laser=num_laser)
            if policy_name == "nbv":
                policy = NextBestViewPolicy(
                    weight,
                    PotentialVisibilityInformationGain(radius=100, rays=nbv_rays),
                    FrontierOcclusionViewpointGenerator(
                        maximum_candidates=nbv_candidates
                    ),
                )
            elif policy_name == "weighted":
                policy = WeightedUtilityPolicy(weight)
            elif policy_name == "nearest":
                policy = NearestFrontierPolicy()
            elif policy_name in ("adaptive", "tuned", "gain_per_cost", "coordinated",
                                  "hybrid_005", "hybrid_010", "hybrid_020",
                                  "intent_nearest", "soft_intent_nearest",
                                  "regret_25", "regret_50", "regret_75",
                                  "intent_only", "trajectory_only", "intent_tight",
                                  "trajectory_050", "trajectory_075", "trajectory_125",
                                  "trajectory_diversified", "recent_trail",
                                  "voronoi_nearest", "frontier_reservation",
                                  "elastic_trajectory", "clearance_utility",
                                  "detour_capped"):
                policy = make_advanced_policy(
                    policy_name, rays=nbv_rays, candidates=nbv_candidates
                )
            else:
                raise ValueError("policy_name no reconocido")
            world = CompetitionWorld(self.maps_root/environment/"occ_map.npy", config, policy)
            while world.timestep < budget:
                world.step()
            recorded_weight = getattr(policy, "weights", weight)
            results.append(TrialResult(recorded_weight.label, recorded_weight.information_gain,
                recorded_weight.travel_cost, recorded_weight.redundancy, recorded_weight.relay_risk,
                environment, size, robots, int(start[0]), int(start[1]),
                budget, world.coverage, policy_name))
        return results


def summarize(results: list[TrialResult]) -> list[dict[str, object]]:
    rows = []
    for policy, label in sorted({(result.policy, result.weights) for result in results}):
        selected = [result for result in results if result.weights == label and result.policy == policy]
        values = [result.coverage for result in selected]
        def avg(predicate):
            subset = [result.coverage for result in selected if predicate(result)]
            return mean(subset) if subset else float("nan")
        rows.append({"Policy":policy, "Weights":label, "Small":100*avg(lambda r:r.size=="Small"),
            "Medium":100*avg(lambda r:r.size=="Medium"), "Large":100*avg(lambda r:r.size=="Large"),
            "Single":100*avg(lambda r:r.robots==1), "Multi":100*avg(lambda r:r.robots>1),
            "Mean":100*mean(values), "Median":100*median(values), "Worst":100*min(values),
            "StdDev":100*pstdev(values), "Trials":len(values)})
    return sorted(rows, key=lambda row:(-float(row["Median"]), -float(row["Worst"])))


def leave_one_environment_out(results: list[TrialResult]) -> list[dict[str, object]]:
    rows = []
    environments = sorted({result.environment for result in results})
    labels_by_environment = [set((result.policy, result.weights) for result in results if result.environment == env)
                             for env in environments]
    common_labels = set.intersection(*labels_by_environment) if labels_by_environment else set()
    if not common_labels:
        return []
    complete = [result for result in results if (result.policy, result.weights) in common_labels]
    for held_out in environments:
        training = [result for result in complete if result.environment != held_out]
        validation = [result for result in complete if result.environment == held_out]
        best_row = summarize(training)[0]
        best = (best_row["Policy"], best_row["Weights"])
        scores = [result.coverage for result in validation if (result.policy, result.weights) == best]
        rows.append({"HeldOut":held_out, "SelectedPolicy":best[0], "SelectedWeights":best[1],
                     "ValidationMean":100*mean(scores), "ValidationWorst":100*min(scores)})
    return rows


def export_csv(rows: list[object], destination: str | Path) -> None:
    path = Path(destination); path.parent.mkdir(parents=True, exist_ok=True)
    dictionaries = [asdict(row) if hasattr(row, "__dataclass_fields__") else dict(row) for row in rows]
    if not dictionaries:
        raise ValueError("No hay resultados para exportar")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=dictionaries[0].keys())
        writer.writeheader(); writer.writerows(dictionaries)
