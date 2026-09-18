# ICUAS 2025 reproduction audit

**Audit date:** 2026-09-17  
**Scope:** establish whether the public repository can reproduce the quantitative
results and video claimed in `paper/ICUAS2025_JackieLee.pdf`, without modifying
the legacy implementation or its saved outputs.

## Conclusion

The checked-in implementation is **not currently a reproducible source** for
Table I, Table II, Figure 8, or a defensible simulation video.  The numeric
tables have no generating experiment or metric implementation in this
repository.  Per the author's 2026-09-17 clarification, the original paper
may not have been run as an actual simulation; these are unverified manuscript
values, not a test oracle for the new code.

The correct next milestone is therefore a clean, independently testable
implementation of the published problem specification.  It must report its
measured values as re-run results, even if they differ from the paper.

## Published reference values

| Method | Coverage (%) | Team score | Redundancy (%) | Average UAV delay (s) |
| --- | ---: | ---: | ---: | ---: |
| Point-based | 57.3 | 8,400 | 28.1 | 38.2 |
| Naive area-based | 85.1 | 15,930 | 21.6 | 42.3 |
| FOV-aware | 92.4 | 18,720 | 13.8 | 27.6 |

The reported ablation values are full / no-FOV / no-reinitialization /
neither = `(92.4, 18,720, 13.8)` / `(86.7, 16,490, 20.3)` /
`(88.1, 17,050, 18.9)` / `(81.2, 14,430, 24.5)`.

## Execution audit

`TOP/main.py` fails before simulation because
`TOP/config_settings.py` references the absent absolute path
`/Users/jackielee/PersistentServailence/environments/corridor_scene/corridor_scene.csv`.
The local relative file is `environments/corridor_scene/corridor_scene.csv`.

The environment itself is usable: Python 3.9.6 imports the declared packages,
and Gurobi 12.0.1 solves a one-variable smoke-test model under its restricted
academic/non-production license (expiry: 2026-11-23).

## Saved legacy output inventory

These files are tracked at the repository's only public-release commit
(`4dea25e`).  Their SHA-256 values were recorded before any code change.

| File | SHA-256 |
| --- | --- |
| `TOP/top_final_results.json` | `1c02529dd45cbc8ec8e368592594368a26d68655a04635af6957bea8a5b5cba3` |
| `TOP/uav_schedule.json` | `80760ec6223f8d1df52074091fa73bd3e92fb16f3dd9fce9593f0d210079ea58` |
| `TOP/uav_simulation_data.json` | `028718d367572b0ed16e7f48d5e8e2aa24b14aabb8d12ff89c991cfa0f90296a` |
| `TOP/ugv_simulation_data.json` | `c99d6798e02373b21dde6302a364f9457bc4d7cce4b5afd90cc8f7c465f615fe` |
| `map/zz_result1.json` | `6ec88f7b6dcc3e1daecdf26d9e572a51640db5986f32a792e605077a44d4733b` |
| `map/uav_simulation_data.json` | `da612e76b24b1eea83bf68eb9e079f8a35fc35affa160dbbbffabd4ab0103752` |
| `map/ugv_simulation_data.json` | `c43e9bb04afab3ca67ba60d2060a142d57cbd1ce62ccf9cc10ee6f00cded2e80` |

The `TOP` saved output has one segment per UAV with `start_time=0` and
`end_time=2`; these are state-change indices mislabelled as times, not evidence
of an actual two-second flight. The file cannot establish a 20-minute sortie
timeline. It stores no coverage, redundancy,
delay, or per-cell visit data.  The `map` output has no flight segments.
Neither can be used to reconstruct the paper tables.

## Traceability findings

- `TOP/pipeline/step_one.py` constructs routes by `random.shuffle`; it has no
  seed and no heading-cone/FOV evaluation.  Its score is a count of waypoints,
  not the paper's FOV coverage score.
- `TOP/help.py` derives a landing location by moving straight in the x
  direction from take-off; it does not follow the UGV's road route.  Its state
  history records only state changes, which explains the two-index segments.
- `TOP/schedule_maxfly.py` is an executable script with hard-coded durations;
  it is not a reusable experiment input and does not record pad assignments or
  delay metrics.
- `map/` is a separate, parallel `z*.py` implementation.  It contains some
  FOV geometry but has incompatible simulation and pipeline interfaces.  Its
  plotting scripts contain illustrative fixed arrays, not Table I/II metric
  generation.
- No source file computes all four Table I metrics or the Table II ablations.
  The values occur in `paper/root.tex` only.  Figure-generation provenance for
  the original trajectory overlay is likewise absent.

## Reimplementation acceptance criteria

Before comparing a new run with the published reference values, the maintained
implementation must provide all of the following.

1. A versioned scenario file specifying the campus AOI, road route, UAV/UGV
   parameters, 3 UAVs, 2 pads, 20-minute flight, and 14-minute charge.
2. A seeded, explicit schedule with pad assignments and a time-indexed
   event log (takeoff, landing, charging start/end, and delay).
3. Time-indexed UAV and UGV trajectories that obey the stated speed, battery,
   flight-duration, and road-route constraints.
4. A single coverage evaluator that defines the 100 m x 67 m footprint,
   AOI mask, unique coverage, redundancy, and team score; the three methods
   and ablations must all use it.
5. Results CSV/JSON containing configuration, seed, software revision,
   metrics, and trajectories; figures and MP4 must be generated only from
   those artifacts.
6. Unit and integration tests for geometric coverage, scheduling feasibility,
   metric calculation, and deterministic repeatability.

## Immediate implementation order

1. Freeze the legacy directories as read-only reference and introduce an
   isolated maintained package plus a small deterministic corridor test case.
2. Implement the common event simulator and the metric evaluator first.
3. Port the fixed-route, FOV-aware planner; then add point-based and naive
   area-based baselines behind the same interfaces.
4. Recreate the campus scenario, run repeated seeded trials, and generate the
   first honest comparison table and video.

Published values will remain visible as unverified manuscript reference only.
A discrepancy is a research result to diagnose, not a value to tune toward.
