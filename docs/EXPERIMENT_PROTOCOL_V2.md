# Frozen evaluation protocol v2 — 2026-09-17

Written before running the physical-comparison v2 experiments. All specified
conditions and seeds must be reported, including unfavorable or null outcomes.
Manuscript numbers are not fitting targets. This is a local protocol record,
not an externally timestamped preregistration.

## Questions and claims

1. Can the proposed area planner execute the paper's UAV/UGV schedule with
   bounded speed/turn rate and exact rendezvous under battery/pad constraints?
2. Does area-aware planning improve unique coverage over point-driven greedy
   routing when BOTH have the SAME physical camera and vehicle dynamics?
3. Does considering heading feasibility during candidate selection help versus
   choosing idealized area-greedy commands and executing them with the SAME
   bounded-turn vehicle controller?
4. Do local search and reward reinitialization help beyond feasible area-greedy
   construction, after accounting for their extra computation?
5. Are coverage gains accompanied by lower observation age? A cell covered
   once is not sufficient evidence of persistent monitoring performance.

No direction of improvement is guaranteed in advance. Only measured supported
claims may appear in the revised research narrative.

## Physical conditions

Same campus AOI, projected road geometry, fixed UGV route and 3-UAV/2-pad ILP
schedule for every method. 10 m/s UAV, 5 m/s UGV, 1200 s flight, 840 s charge,
100×67 m camera, all cell priorities one. **Every executed UAV trajectory is
limited to 5°/s**, including baselines and heading-planning ablations.
All use the same exact-time return/loiter mechanism and report its coverage.

Mission length is 6120 s: one 34-minute startup cycle plus two further cycles.
Report cumulative coverage from t=0 and observation-age metrics over both the
whole mission and the post-startup window [2040,6120). Age is initialized to
zero at t=0 everywhere; unobserved cells age continuously and are not excluded.
Planning grid 5 m, reporting grid 1 m, 1-second camera exposure sampling.

Initial heading remains zero. Battery is an ideal linear duty-cycle model;
instantaneous launch/landing, no disturbances, collision avoidance, wind or
UGV battery model are claimed. AOI is a sensing region, not a geofence.

## Methods and controls

- `point`: nearest unvisited sparse targets, revisited cyclically when all
  targets have been visited; target visits shared across sorties.
- `naive`: greedily selects from unrestricted instantaneous-heading camera
  candidates, then executes the command after clipping to the common turn
  limit. It ignores turning cost in ranking, not in physical execution.
- `sweep`: deterministic footprint-spaced parallel-strip survey, with strips
  divided among vehicles. Arc/tangent connections and the common return rule
  impose the same motion constraints. This is a basic lawnmower control, not
  a claim to implement the full optimal boustrophedon decomposition literature.
- `greedy`: feasible heading-aware FOV greedy construction; no local search
  and no reward reinitialization. Strongest direct control for added optimizer
  stages, using exactly the proposed construction code.
- `full`: greedy construction + local operators + reinitialization.
- `no_heading`: removes heading-aware candidate ranking, retains the same
  physical turn limit and optimizer stages.
- `no_reinit`: skips only the reinitialization stage.
- `neither`: disables heading-aware ranking and reinitialization, retaining
  local search.

Every baseline's real executed footprint is evaluated. The v1 unrestricted
180°/s trajectories remain historical idealized controls, not fair physical
comparisons. All routes are evaluated by a common measurement module.

## Fixed experiment matrix and seeds

- Main: eight methods × seeds 0–9, all on the same 6120-second mission.
- Heading sensitivity: physical turn limits 2.5°, 5°, 10° per second, methods
  full/greedy/naive, seeds 0–2. The 5° cases reuse main-run results.
- Grid sensitivity: full/greedy/naive, planning resolution 1 m, seed 0; compare
  to 5 m main cases. Evaluation remains 1 m for all.
- Videos: fixed seed 0, main full/greedy/naive, and a detailed full-method
  view. Selection occurs before seeing v2 results, not by best-looking outcome.

Search settings retain the v1 two outer operator passes and one reinitialization
pass. No condition or seed is silently removed after inspection. Reproducible
code bugs may be corrected; affected batches must be rerun and the correction
documented. Hypothesis-driven algorithm changes require a new protocol version.

## Measurements and inference

- Unique coverage %, covered area, time-to-80/95% (unreached = right-censored).
- Area/time-averaged observation age and terminal spatial p95 age, in seconds.
  Full-window and post-startup means are both reported. Observation resets age
  to zero; this is an explicit freshness metric, not the paper's unknown score.
- Completed blind intervals and terminal/max censored unobserved time.
- Exposure redundancy AND distinct observation-episode revisits. Their
  denominators and camera sampling rate are retained from the specification.
- Measured rendezvous error, turn rate, pad use, battery bounds, airborne time,
  charging delay and computation time. Feasible ideal schedules should have
  zero queue delay; no delay-superiority claim is expected without disturbances.
- Report mean, sample SD and paired seed-level differences with bootstrap
  95% intervals (fixed bootstrap seed). These describe this scenario and this
  small seed sample, not universal performance or uncertainty across worlds.
- Deterministic baselines are identified explicitly; repeated identical runs
  do not create independent evidence. Runtime is descriptive and affected by
  shared-machine load, not a controlled hardware benchmark.

## Literature grounding

Persistent monitoring concerns time-varying information/field accumulation,
not merely one-time geometric union. Smith, Schwager and Rus (2012) study
growing/decreasing fields along repeated robot paths; our age-reset metric is
a different explicit diagnostic, not their controller or theorem:
[author-hosted paper](https://msl.stanford.edu/papers/smith_persistent_2012.pdf).

Parallel-strip coverage is a standard comparison family; the CMU work describes
boustrophedon decomposition, while our simple strip control does not claim its
complete algorithm:
[Choset and Pignon](https://publications.ri.cmu.edu/coverage-path-planning-the-boustrophedon-decomposition).

Sensor coverage paths can contain turns incompatible with nonholonomic vehicle
motion; this motivates enforcing execution dynamics equally:
[Paull et al., author-hosted paper](https://people.csail.mit.edu/lpaull/publications/Paull_CYB_2014.pdf).
