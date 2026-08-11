# Policy comparison log

## Protocol A — design screening

- Environments: env1, env3, env6
- Robots: 3
- Start: (15, 15)
- Steps: 20 (proxy)
- LiDAR rays: 32; NBV rays: 16; NBV candidates: 20

NBV beat Nearest in this very short screening, but the result reversed at 50
steps. This protocol is retained only as design evidence.

## Protocol B — 50-step validation

- Environments: env1, env3, env6
- Robots: 3 and 5
- Start: (15, 15)
- Steps: 50 (proxy)
- LiDAR rays: 64; NBV rays: 24; NBV candidates: 40

| Policy | Mean % | Median % | Worst % |
|---|---:|---:|---:|
| Intent-aware Nearest | 1.8444 | 1.0422 | 1.0341 |
| Soft Intent Nearest | 1.8357 | 1.0462 | 1.0341 |
| Nearest | 1.8347 | 1.0545 | 1.0230 |
| Tuned NBV | 1.4800 | 1.0357 | 0.4732 |

### Iteration: compact trajectory exclusion

Using only trajectory history and reducing its exclusion radius from 5 m to
2.5 m produced the current winner under the same 50-step protocol:

| Policy | Mean % | Median % | Worst % |
|---|---:|---:|---:|
| **Trajectory Diversified** | **1.9616** | **1.0484** | **1.0341** |
| Intent-aware Nearest | 1.8444 | 1.0422 | 1.0341 |
| Nearest | 1.8347 | 1.0545 | 1.0230 |

This is a 6.35% relative mean improvement over Intent-aware Nearest and a
6.92% improvement over Nearest. Evidence is in
`trajectory_radius_validation/`. Official-budget validation remains required.

Intent-aware Nearest is the current candidate: it improves mean and worst-case
over Nearest and substantially beats NBV under this protocol. It does not beat
Nearest on every individual map, and these reduced-fidelity results are not an
official-budget claim. Raw evidence lives in `intent_policy_validation/`,
`soft_intent_validation/`, `policy_comparison_validation/`, and
`nbv_weight_iteration/`.

## Iteration: three coordination competitors

Three alternatives were added to challenge Trajectory Diversified:

- Recent Trail excludes only the most recent peer trajectory samples.
- Voronoi Nearest assigns frontiers by proximity to the observing robot.
- Frontier Reservation combines short trail avoidance with peer intent reservations.

The 20-step screening (`three_policy_screening/`) selected Voronoi Nearest:

| Policy | Mean % | Median % | Worst % |
|---|---:|---:|---:|
| Voronoi Nearest | 0.7542 | 0.6482 | 0.4253 |
| Recent Trail | 0.7186 | 0.5425 | 0.4243 |
| Trajectory Diversified | 0.7186 | 0.5425 | 0.4243 |
| Frontier Reservation | 0.7183 | 0.5425 | 0.4233 |

Under the unchanged 50-step validation protocol, Voronoi Nearest scored a
1.8347% mean, 1.0545% median, and 1.0230% worst case. It therefore did not
displace Trajectory Diversified (1.9616% mean). This reversal reinforces that
short screenings are useful for pruning, not for declaring a winner. Raw
validation evidence is in `voronoi_validation/`.

## Iteration: adaptive trajectory competitors

Three additional policies target limitations of the fixed 2.5 m exclusion:

- Elastic Trajectory grows its exclusion radius from 2.0 to 3.0 m with map maturity.
- Clearance Utility rewards separation continuously and hard-excludes only below 1.5 m.
- Detour Capped accepts diversification only within a 3.0 m travel detour.

In the 20-step screening (`second_three_policy_screening/`), Detour Capped
improved mean coverage from 0.7186% to 0.7417% (+3.22% relative). Elastic
Trajectory scored 0.7183% and Clearance Utility 0.7138%.

The unchanged 50-step validation (`detour_capped_validation/`) did not confirm
the early advantage: Detour Capped reached 1.9574% mean versus 1.9616% for
Trajectory Diversified, with a 1.0470% median and 1.0230% worst case. The three
remain experimental options; Trajectory Diversified remains the validated
proxy champion.

## Return-policy comparison

Return timing and relay handoff are now independent from exploration. The
comparison fixes Trajectory Diversified as the exploration policy and evaluates
Periodic (legacy), Deadline, Payload Adaptive, and Link Aware return policies.

An 80-step screening on env1/env3 with three robots and a 25-step baseline
relay period produced:

| Return policy | Reported mean % | Worst % | Robots at base (mean) |
|---|---:|---:|---:|
| Periodic | 2.9556 | 1.9557 | 1.0 |
| Payload Adaptive | 2.5838 | 1.2712 | 3.0 |
| Deadline | 2.5810 | 1.2705 | 3.0 |
| Link Aware | 2.4884 | 1.1688 | 3.0 |

Periodic maximized reported coverage in this proxy. The new policies traded
coverage for a guaranteed physical return of all robots. Raw results are in
`return_policy_screening/`; a longer official-budget run is still required.

### Iteration: efficient return timing

Three policies replace the conservative final margin with an exact A* deadline:

- Efficient Periodic retains scheduled deliveries.
- Selective Courier performs a scheduled delivery only with useful payload.
- Value Density compares deliverable cells against the A* return cost.

Under the same 80-step protocol, all three returned 3/3 robots and cleared all
pending cells. Selective Courier and Value Density reached 2.7747% mean reported
coverage (1.5940% worst); Efficient Periodic reached 2.7685% (1.5083% worst).
They improve the prior safe-return winner, Payload Adaptive (2.5838%), by 7.39%
and 7.15% respectively. Legacy Periodic still leads raw coverage at 2.9556%,
but returned only 1/3 robots on average. Evidence is in
`improved_return_screening/` and `improved_return_tuned_candidates/`.

### Iteration: frontier-assisted return

Three return planners can now spend a tightly bounded final detour observing a
frontier before completing the A* route to base: Nearest Frontier Return, Gain
Sweep Return, and Homeward Sweep Return. Every composite route is rejected
unless `robot -> frontier -> base` fits in the exact remaining movement budget.
Return paths are cached per robot and only replanned if invalidated.

Wide 8--12 step windows were rejected as inefficient. With tuned 2--3 step
windows (`frontier_return_tight_screening/`), Nearest Frontier Return and
Homeward Sweep Return scored 2.7096% mean reported coverage; Gain Sweep Return
scored 2.7027%. All returned 3/3 robots and cleared pending information. They
beat Payload Adaptive, Deadline, and Link Aware, but do not displace Selective
Courier at 2.7747% under this proxy.

## Handoff-policy comparison

Handoff decisions are now independent from both exploration and return timing.
Payload Progress transfers only when the combined pending payload is at least
300 cells, the receiver has gained at least 1.5 m toward base, is within the
communication range, and is not moving away from base. Candidate utility
rewards base progress and penalizes meeting distance and outward motion.

With Trajectory Diversified exploration and Efficient Periodic return fixed,
the 80-step env1/env3 screening produced identical 2.7685% mean reported
coverage, 1.5083% worst coverage, zero pending cells, and 3/3 returned robots
for both handoff policies. Closest Progress performed 2.5 handoffs per trial;
Payload Progress performed zero. The new policy therefore removed transfers
that provided no measurable benefit under this protocol. Evidence is in
`handoff_policy_screening/`.

### Iteration: utility-aware handoffs

Three policies extend the payload gate with distinct receiver utilities:

- Time Saving maximizes net delivery-time reduction.
- Returning Courier requires observed motion or intent toward base.
- Link Quality adds wall attenuation and meeting-distance penalties.

The handoff transfer was also corrected to move responsibility to the receiver
instead of duplicating the payload on both robots. In a five-robot, 100-step
env1 proxy with actual handoff opportunities, Payload Progress scored 6.9410%.
Returning Courier and guarded Time Saving scored 7.1373% (+2.83% relative),
while guarded Link Quality scored 7.1395% (+2.86%) with only two handoffs. All
policies returned 5/5 robots and left zero pending cells. Evidence is in
`advanced_handoff_five_robot_proxy/` and `advanced_handoff_guarded_proxy/`.

## Joint three-method search

A Cartesian search now evaluates exploration (`decide`), return timing
(`should_relay`), and relay transfer (`decide_relay_handoff`) as complete
combinations. SafeScore is reported coverage multiplied by the fraction of
robots physically recovered; pending cells and handoff count break ties.

The 27-combination coarse search selected Trajectory Diversified + Selective
Courier. It scored 2.0985% with 5/5 robots at base, versus 2.0201% for the best
Detour Capped combination. An env1/env3 validation at 80 steps produced 5.4738%
and 1.0703% respectively, always with 5/5 robots and zero pending cells.
Handoff policies tied because Selective Courier did not request an intermediate
relay in these trials. Link Quality is retained for the final combination based
on its separate handoff-opportunity validation, where it beat Payload Progress.

The complete official-style submission is
`Policies/competition_best_combination.py`. Raw joint rankings are in
`combination_coarse_search/` and `combination_handoff_validation/`.
