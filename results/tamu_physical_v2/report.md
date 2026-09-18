# Physical comparison v2: all prespecified results

101 runs: 80 main, 18 turn-limit sensitivity, 3 planning-grid sensitivity.
Same physical turn limit applies to every method in each condition.

| Method | Coverage % ± SD | Post-startup age, min ± SD | Revisit % | Planning s |
| --- | ---: | ---: | ---: | ---: |
| Point planner | 88.212 ± 0.000 | 32.95 ± 0.00 | 75.91 | 0.66 |
| Naive area | 99.333 ± 0.396 | 24.75 ± 1.75 | 90.46 | 5.29 |
| Parallel-strip survey | 76.417 ± 0.000 | 30.01 ± 0.00 | 91.34 | 0.65 |
| Feasible area greedy | 99.550 ± 0.069 | 22.70 ± 0.63 | 91.21 | 1.93 |
| FOV + reinitialization | 99.629 ± 0.115 | 22.72 ± 0.67 | 92.16 | 4.55 |
| No heading-aware ranking | 99.502 ± 0.156 | 24.25 ± 0.89 | 91.48 | 11.69 |
| No reinitialization | 99.554 ± 0.073 | 22.70 ± 0.64 | 91.10 | 2.49 |
| Neither component | 99.344 ± 0.365 | 24.68 ± 1.57 | 90.62 | 5.86 |

## Paired differences: full minus each control

Coverage: positive favors full. Observation age: negative favors full.
Intervals are percentile bootstrap 95% intervals over ten paired seeds.

| Control | Metric | Difference | 95% interval |
| --- | --- | ---: | ---: |
| point | coverage_percent | 11.417 | [11.350, 11.484] |
| point | post_warmup_mean_age_s | -614.102 | [-637.795, -590.282] |
| naive | coverage_percent | 0.296 | [0.112, 0.518] |
| naive | post_warmup_mean_age_s | -122.155 | [-198.563, -58.950] |
| sweep | coverage_percent | 23.212 | [23.145, 23.279] |
| sweep | post_warmup_mean_age_s | -437.909 | [-461.601, -414.088] |
| greedy | coverage_percent | 0.079 | [0.031, 0.129] |
| greedy | post_warmup_mean_age_s | 1.099 | [-3.162, 6.377] |
| no_heading | coverage_percent | 0.127 | [0.005, 0.257] |
| no_heading | post_warmup_mean_age_s | -91.889 | [-137.843, -51.537] |
| no_reinit | coverage_percent | 0.075 | [0.029, 0.123] |
| no_reinit | post_warmup_mean_age_s | 1.224 | [-3.224, 6.484] |
| neither | coverage_percent | 0.285 | [0.111, 0.487] |
| neither | post_warmup_mean_age_s | -118.111 | [-187.469, -58.926] |

## Time to coverage thresholds

Means include reached runs only; unreached runs are reported, not assigned a favorable value.

| Method | 80%: reached / 10 | Mean min if reached | 95%: reached / 10 | Mean min if reached |
| --- | ---: | ---: | ---: | ---: |
| point | 10 | 75.60 | 0 | unreached |
| naive | 10 | 53.12 | 10 | 79.81 |
| sweep | 0 | unreached | 0 | unreached |
| greedy | 10 | 50.68 | 10 | 67.67 |
| full | 10 | 50.68 | 10 | 67.67 |
| no_heading | 10 | 52.23 | 10 | 78.79 |
| no_reinit | 10 | 50.68 | 10 | 67.67 |
| neither | 10 | 53.06 | 10 | 79.81 |

## Heading-limit sensitivity (all seeds 0–2)

| deg/s | Method | Coverage % | Post-startup age, min |
| ---: | --- | ---: | ---: |
| 2.5 | full | 96.267 | 28.41 |
| 2.5 | greedy | 95.644 | 28.55 |
| 2.5 | naive | 95.228 | 28.36 |
| 5 | full | 99.716 | 22.97 |
| 5 | greedy | 99.574 | 22.99 |
| 5 | naive | 99.490 | 24.75 |
| 10 | full | 99.990 | 20.17 |
| 10 | greedy | 99.806 | 22.08 |
| 10 | naive | 99.790 | 26.05 |

## Planning-grid sensitivity (seed 0, 1 m evaluation for both)

| Method | 5 m planner coverage % | 1 m planner coverage % |
| --- | ---: | ---: |
| full | 99.639 | 99.673 |
| greedy | 99.548 | 99.666 |
| naive | 99.481 | 73.032 |

Point and sweep controls are deterministic; identical repeated runs are not independent evidence.
All scenarios and seeds are retained. Confidence intervals describe seed variability on this map only.
Computation times are descriptive: jobs shared the same workstation.
Unique coverage is not a guarantee of persistent freshness. Observation age includes never-observed cells.
No claim is made that the model is collision-free, robust to disturbances, or flight-test validated.
