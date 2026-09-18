# Measured campus experiments — 2026-09-17

All numbers below come from executable simulations and their saved artifacts.
They are not reconstructions of the manuscript's unverified tables.

## Run definition

- Campus AOI: 1.726529 km² from `map/mapofroads.geojson`.
- Fixed UGV route: 3902.126 m out-and-back road walk. Source roads have seven
  disconnected components; the largest road-length component is used without
  adding fictitious connectors. Ordered vertices are in every result.
- Three UAVs, two pads, 1200 s flight / 840 s charge, UAV 10 m/s, UGV 5 m/s.
- Horizon 4080 s including initial charging; six launched sorties, five
  completed before the horizon, 6360 total airborne vehicle-seconds.
- Planning grid 5 m; reporting grid 1 m (1,726,520 interior cell centres);
  camera 100×67 m sampled every second.
- Six methods × seeds 0–4 = 30 runs. SD means sample standard deviation, not
  a confidence interval. The point heuristic is deterministic, so its five
  repetitions have zero seed variance and are not independent evidence.

## Measurements

| Method | Coverage % (mean ± SD) | Revisited covered cells % | Charging delay s |
| --- | ---: | ---: | ---: |
| Point-driven greedy | 71.98 ± 0.00 | 48.29 | 0.0 |
| Naive area, unrestricted heading | 99.73 ± 0.12 | 89.45 | 0.0 |
| FOV + reward reinitialization | 96.25 ± 1.44 | 75.47 | 0.0 |
| No heading constraint | 99.88 ± 0.08 | 94.09 | 0.0 |
| No reward reinitialization | 96.03 ± 1.54 | 75.28 | 0.0 |
| Neither component, local search retained | 99.73 ± 0.12 | 89.45 | 0.0 |

The full method covers more area than the point-driven baseline, but less than
the unrestricted-heading baseline. It achieves about 13.98 percentage points
less revisited-cell fraction than naive area while respecting the 5°/s heading
limit. The unrestricted controls turn up to 180° per second; their larger
coverage is not evidence of a physically equivalent flight solution.

The manuscript's claim that heading constraints always increase area coverage
is not supported by this experiment. Reduced revisits also cannot by itself
establish better persistent surveillance; revisit intervals and priority
requirements need an explicit mission objective before making that claim.

Reward reinitialization increases mean coverage by only 0.22 percentage points
on these five paired seeds. For the full method, only seed 1 improves here.
Across all local-search/ablation runs, accepted proposals were: two-point
exchange 0/40, one-point movement 2/40, 2-opt 2/40, reinitialization 10/60.
The effect of the local operators is modest in this bounded search; no
parameters were retuned to force a desired ranking. This implementation uses
feasible guidance-waypoint decoding, as detailed in the specification.

Charging delay is calculated from actual landing and charging events and is
zero for all methods. The shared schedule already satisfies pad capacity, and
the model has no arrival disturbance. Algorithm-specific nonzero delay gains
would require a separate disturbance model.

## Accuracy and repeatability checks

- All 30 saved artifacts passed independent checks of metrics, hashes,
  trajectory endpoints, battery evolution, grounded positions and pad use.
- Maximum rendezvous error across the batch: 1.51e-11 m (numerical roundoff).
  Constrained trajectories satisfy 5°/s; sample chord speeds do not exceed
  10 m/s. Pad occupancy never exceeds two.
- Full-method seed 0 repeated in a separate run produces identical numerical
  trajectory/state/coverage arrays and identical metrics except runtime.
- Seed 0, 1 m **planning** grid: coverage 95.77335%, versus 95.91902% with
  5 m planning. Both were measured at 1 m. This one-seed difference of
  -0.14567 percentage points is a sensitivity check, not a convergence proof.
- A half-second interpolated-pose diagnostic gives 96.10256%, +0.18355
  percentage points versus 1 Hz. This uses linear position interpolation and
  shortest-arc heading interpolation; it is not an exact 60 fps camera run.
  Exposure-based redundancy should not be compared across sample rates.

## Artifacts and regeneration

- `results/tamu_v1/metrics.csv`: every run's actual metrics.
- `results/tamu_v1/summary.{md,json}`: grouped summary.
- `results/tamu_v1/verification.json`: independent saved-artifact checks.
- `results/tamu_v1/{method}_seed{0..4}/{result.json,simulation.npz}`:
  configuration, events, trajectories, energy, occupancy and observation data.
- `results/tamu_v1/figures/comparison.mp4`: 30 s, H.264, 1800×620, 12 fps;
  the complete 68-minute mission, three methods side by side, seed 0 chosen
  in advance rather than the best-performing seed.
- `results/tamu_v1/figures/{trajectories,coverage,coverage_over_time}.png`.
- `results/tamu_sensitivity/planning1m_seed0/`: 1 m planner run.
- `results/tamu_v1/full_seed0/sampling_sensitivity.json`: sampling diagnostic.
- `results/tamu_v1/source_snapshot/`: exact Python source of the numerical
  batch, SHA-256 `35fbd4d17a3bcbd9756bc52d050e48a4a5cf6cfbbe0e35a0e7b08c5dc5b6696f`.
  Subsequent display-text and empty-horizon handling edits do not affect these
  measurements. The source hash is reconstructed from sorted Python filenames
  followed by file contents, as in `experiment.py`.

Commands are in the root README. Raw artifacts are local, regenerable and
git-ignored; this report and the scenario are versionable source material. See
`implementation_spec.md` for all manuscript ambiguities and modeling choices.
