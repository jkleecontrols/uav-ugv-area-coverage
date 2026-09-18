# ICUAS UAV–UGV area coverage: executable reimplementation

Actual simulations of the problem described in **Team Orienteering and Scheduling
Algorithms for Collaborative UAV-UGV Area Coverage with Battery Constraints**
(Lee and Rathinam, ICUAS 2025).

The manuscript's old numerical tables are **unverified**. The maintained code
produces new measurements from stored trajectories, battery states and charging
events. Read the [implementation choices](docs/implementation_spec.md) and
[legacy audit](docs/reproduction_audit.md) before interpreting comparisons.
The completed current experiment is **v2: 101 runs with identical physical
constraints for every method**, not the historical v1 unrestricted controls.
Start with the [Korean advisor brief](docs/PROFESSOR_BRIEF_KO.md) and the
[full measured report](results/tamu_physical_v2/report.md).

Mean coverage: full 99.629%, feasible greedy 99.550%, naive 99.333%, point
88.212%. Added optimization gains are small; no freshness improvement over
feasible greedy is established. All conditions, including unfavorable grid
sensitivity, are retained. See the report before quoting a performance claim.

- 60-second comparison and full-method detail videos: MP4s are not committed;
  regenerate them locally with the `render` commands below.
- [Frozen protocol](docs/EXPERIMENT_PROTOCOL_V2.md) and [model limitations](docs/MODEL_AND_CLAIMS.md)

The first 30-run batch is historical: [v1 measured report](docs/measured_results.md).

## Run

The existing `.venv` is usable. For a fresh environment:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-repro.txt
```

Gurobi is required; the default 51-variable schedule fits its restricted license.
MP4 export requires `ffmpeg` on PATH.
`requirements-repro.txt` pins the tested simulation dependencies;
`requirements.txt` also includes dependencies used by legacy scripts.

```bash
# Geometry, scheduling, event and integration checks
.venv/bin/python -m unittest discover -s tests -v

# One physical v2 102-minute mission (use a new output directory)
.venv/bin/python -m icuas_repro run --scenario scenarios/tamu_physical_v2.json \
  --method full --seed 0 --output results/my_run

# All 101 protocol runs; existing matching-version results can be resumed
.venv/bin/python scripts/run_protocol_v2.py --root results/tamu_physical_v2 --phase main
.venv/bin/python scripts/run_protocol_v2.py --root results/tamu_physical_v2 --phase sensitivity --seeds 0 1 2
.venv/bin/python scripts/run_protocol_v2.py --root results/tamu_physical_v2 --phase grid
.venv/bin/python scripts/analyze_protocol_v2.py results/tamu_physical_v2

# Independent verification of saved metrics, trajectories and energy
.venv/bin/python scripts/check_results.py results/tamu_physical_v2/main
.venv/bin/python scripts/check_results.py results/tamu_physical_v2/turn_2.5
.venv/bin/python scripts/check_results.py results/tamu_physical_v2/turn_10
.venv/bin/python scripts/check_results.py results/tamu_physical_v2/planning_1m

# Plots and a 60-second video of the entire mission
.venv/bin/python -m icuas_repro render \
  results/tamu_physical_v2/main/naive_seed0 \
  results/tamu_physical_v2/main/greedy_seed0 \
  results/tamu_physical_v2/main/full_seed0 \
  --output results/tamu_physical_v2/video --video --frames 720

.venv/bin/python scripts/render_detail.py results/tamu_physical_v2/main/full_seed0 \
  --output results/tamu_physical_v2/video_detail
```

On a restricted filesystem, set `MPLCONFIGDIR` and `XDG_CACHE_HOME` to writable
cache directories. `compare` resumes existing outputs in its root; use a new
root when changing scenario or implementation. The artifact checker rejects
mixed source versions.

## Scenario and outputs

`scenarios/tamu_physical_v2.json` declares 3 UAVs, 2 charging pads, 20-minute flights,
14-minute charging, UAV/UGV speeds 10/5 m/s and a 100×67 m camera footprint.
The supplied campus GeoJSON supplies both the AOI and roads. The selected fixed
road route and all parameters are saved in each result.

Each run contains:

- `result.json`: measured metrics, configuration, seed, ILP schedule, charging
  intervals, sortie events, search decisions, validation and provenance hashes.
- `simulation.npz`: per-second positions, headings, states, battery, pad
  assignments, per-cell exposures/episodes, coverage curve and full sorties.

V2 analysis adds `all_metrics.csv`, `analysis.json`, `report.md`; rendering produces
trajectory and coverage plots, a video preview and `comparison.mp4`. `results/`
is regenerable and git-ignored, except for the small v2 numerical summaries
(`report.md`, `all_metrics.csv`, `analysis.json`, `manifest.json` and the
`verification.json` files), which are committed so the cited evidence resolves.
Per-run `result.json`, `simulation.npz` and MP4s stay local. Video frames come
from saved simulation data.

Coverage, exposure overlap, revisited-cell percentage and charging delay have
explicit definitions in the specification. Every planner is measured with the
same camera. **All v2 executed methods enforce the same 5°/s heading limit.**
The no-heading ablation removes heading-aware candidate ranking, not physics.
Default planning grid is 5 m, while reported coverage is evaluated at 1 m.
Observation age includes never-observed cells and is reported separately from
unique coverage. The legacy default `scenarios/tamu.json` and generic `compare`
command use v1 semantics; use the explicit v2 scenario/protocol commands above.
The frozen numerical renderer retains a v1 `no_heading` label; the v2 analysis
corrects it to “No heading-aware ranking.” Current videos only show naive,
greedy and full, so do not use that old label for a new v2 ablation figure.

## Maintained code

```
icuas_repro/
  scenario.py     GeoJSON projection and road-only fixed route
  scheduling.py   Periodic charging/robot-admission ILP
  route.py        Closed road-polyline motion
  simulation.py   Timed sortie rendezvous
  planner.py      FOV construction, TOP operators, reinitialization, return
  raster.py       Shared local-window footprint rasterization
  experiment.py   Mission states, battery, metrics, validation and artifacts
  render.py       Result tables, figures and MP4
tests/           Unit and integrated correctness checks
scripts/         Saved-artifact verification
docs/            Audit, implementation assumptions and experiment report
```

`TOP/`, `map/z*.py`, old plot scripts and `UAV_UGV_Simulation/` remain historical
references. Their saved JSON and illustrative figures are not the new results.
The AIAA/DEVCOM TOP operators informed the new area-reward search; the RA-L
periodic scheduling model supplies the charging formulation.

## Related work

- Persistent robot charging (IEEE RA-L 2025): [persistent-robot-charging](https://github.com/jkleecontrols/persistent-robot-charging)

## Citation

```bibtex
@inproceedings{lee2025icuas,
  title     = {Team Orienteering and Scheduling Algorithms for Collaborative {UAV-UGV} Area Coverage with Battery Constraints},
  author    = {Lee, Jaekyung Jackie and Rathinam, Sivakumar},
  booktitle = {International Conference on Unmanned Aircraft Systems (ICUAS)},
  year      = {2025}
}
```
