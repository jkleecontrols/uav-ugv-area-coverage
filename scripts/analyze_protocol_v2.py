"""Analyze every frozen v2 condition; no best-seed or favorable-case filtering."""
import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from icuas_repro.render import LABELS
from run_protocol_v2 import METHODS

# V2 removes heading-aware ranking, NEVER the physical execution constraint.
LABELS = {**LABELS, 'no_heading': 'No heading-aware ranking'}

METRICS = ['coverage_percent', 'post_warmup_coverage_percent', 'mean_observation_age_s',
           'post_warmup_mean_age_s', 'terminal_p95_age_s',
           'revisited_covered_cells_percent', 'planning_runtime_s', 'runtime_s']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root
    manifest = json.loads((root/'manifest.json').read_text())
    expected = [('main', method, seed) for method in METHODS for seed in range(10)]
    expected += [(f'turn_{turn:g}', method, seed) for turn in [2.5, 10.]
                 for method in ['full', 'greedy', 'naive'] for seed in range(3)]
    expected += [('planning_1m', method, 0) for method in ['full', 'greedy', 'naive']]
    rows = []
    runs = {}
    for condition, method, seed in expected:
        path = root/condition/f'{method}_seed{seed}'/'result.json'
        data = json.loads(path.read_text())  # missing run is a hard error
        assert data['provenance']['source_sha256'] == manifest['source_sha256']
        assert data['provenance']['protocol_sha256'] == manifest['protocol_sha256']
        assert data['config']['physical_heading_limit']
        assert data['validation']['all_constraints_passed']
        assert data['validation']['max_heading_change_deg'] <= data['config']['heading_step_deg']+1e-6
        runs[condition, method, seed] = data
        rows.append({'condition': condition, 'method': method, 'seed': seed, **data['metrics']})
    with (root/'all_metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summaries = {}
    for method in METHODS:
        values = [runs['main', method, seed]['metrics'] for seed in range(10)]
        item = {'n': 10, 'deterministic_baseline': method in ('point', 'sweep')}
        for metric in METRICS:
            arr = np.array([v[metric] for v in values])
            item[metric] = {'mean': float(arr.mean()), 'sd': float(arr.std(ddof=1))}
        for threshold in (80, 95):
            key = f'time_to_{threshold}_percent_s'
            times = [v[key] for v in values if v[key] is not None]
            item[key] = {'reached_n': len(times), 'unreached_n': 10-len(times),
                         'mean_among_reached_s': float(np.mean(times)) if times else None}
        summaries[method] = item
    rng = np.random.default_rng(20260917)
    boot_indices = rng.integers(0, 10, size=(10000, 10))
    paired = []
    for control in METHODS:
        if control == 'full':
            continue
        for metric in ['coverage_percent', 'post_warmup_mean_age_s']:
            delta = np.array([runs['main', 'full', seed]['metrics'][metric]-runs['main', control, seed]['metrics'][metric]
                              for seed in range(10)])
            estimates = delta[boot_indices].mean(axis=1)
            low, high = np.percentile(estimates, [2.5, 97.5])
            paired.append({'comparison': f'full - {control}', 'control': control, 'metric': metric,
                           'mean_difference': float(delta.mean()), 'ci95_low': float(low), 'ci95_high': float(high),
                           'paired_seed_differences': delta.tolist(), 'bootstrap_seed': 20260917,
                           'bootstrap_replicates': 10000})
    report = {'complete': True, 'run_count': len(rows), 'main': summaries, 'paired_differences': paired,
              'source_sha256': manifest['source_sha256'], 'protocol_sha256': manifest['protocol_sha256']}
    (root/'analysis.json').write_text(json.dumps(report, indent=2)+'\n')
    lines = ['# Physical comparison v2: all prespecified results', '',
             '101 runs: 80 main, 18 turn-limit sensitivity, 3 planning-grid sensitivity.',
             'Same physical turn limit applies to every method in each condition.', '',
             '| Method | Coverage % ± SD | Post-startup age, min ± SD | Revisit % | Planning s |',
             '| --- | ---: | ---: | ---: | ---: |']
    for method in METHODS:
        item = summaries[method]
        c, a, r, p = [item[key] for key in ['coverage_percent', 'post_warmup_mean_age_s',
                                         'revisited_covered_cells_percent', 'planning_runtime_s']]
        lines.append(f'| {LABELS[method]} | {c["mean"]:.3f} ± {c["sd"]:.3f} | '
                     f'{a["mean"]/60:.2f} ± {a["sd"]/60:.2f} | {r["mean"]:.2f} | {p["mean"]:.2f} |')
    lines += ['', '## Paired differences: full minus each control', '',
              'Coverage: positive favors full. Observation age: negative favors full.',
              'Intervals are percentile bootstrap 95% intervals over ten paired seeds.', '',
              '| Control | Metric | Difference | 95% interval |', '| --- | --- | ---: | ---: |']
    for row in paired:
        lines.append(f'| {row["control"]} | {row["metric"]} | {row["mean_difference"]:.3f} | '
                     f'[{row["ci95_low"]:.3f}, {row["ci95_high"]:.3f}] |')
    lines += ['', '## Time to coverage thresholds', '',
              'Means include reached runs only; unreached runs are reported, not assigned a favorable value.', '',
              '| Method | 80%: reached / 10 | Mean min if reached | 95%: reached / 10 | Mean min if reached |',
              '| --- | ---: | ---: | ---: | ---: |']
    for method in METHODS:
        a = summaries[method]['time_to_80_percent_s']
        b = summaries[method]['time_to_95_percent_s']
        ta = f'{a["mean_among_reached_s"]/60:.2f}' if a['mean_among_reached_s'] is not None else 'unreached'
        tb = f'{b["mean_among_reached_s"]/60:.2f}' if b['mean_among_reached_s'] is not None else 'unreached'
        lines.append(f'| {method} | {a["reached_n"]} | {ta} | {b["reached_n"]} | {tb} |')
    lines += ['', '## Heading-limit sensitivity (all seeds 0–2)', '',
              '| deg/s | Method | Coverage % | Post-startup age, min |', '| ---: | --- | ---: | ---: |']
    for turn in [2.5, 5., 10.]:
        condition = 'main' if turn == 5 else f'turn_{turn:g}'
        for method in ['full', 'greedy', 'naive']:
            vals = [runs[condition, method, seed]['metrics'] for seed in range(3)]
            lines.append(f'| {turn:g} | {method} | {np.mean([v["coverage_percent"] for v in vals]):.3f} | '
                         f'{np.mean([v["post_warmup_mean_age_s"] for v in vals])/60:.2f} |')
    lines += ['', '## Planning-grid sensitivity (seed 0, 1 m evaluation for both)', '',
              '| Method | 5 m planner coverage % | 1 m planner coverage % |', '| --- | ---: | ---: |']
    for method in ['full', 'greedy', 'naive']:
        lines.append(f'| {method} | {runs["main", method, 0]["metrics"]["coverage_percent"]:.3f} | '
                     f'{runs["planning_1m", method, 0]["metrics"]["coverage_percent"]:.3f} |')
    lines += ['', 'Point and sweep controls are deterministic; identical repeated runs are not independent evidence.',
              'All scenarios and seeds are retained. Confidence intervals describe seed variability on this map only.',
              'Computation times are descriptive: jobs shared the same workstation.',
              'Unique coverage is not a guarantee of persistent freshness. Observation age includes never-observed cells.',
              'No claim is made that the model is collision-free, robust to disturbances, or flight-test validated.']
    (root/'report.md').write_text('\n'.join(lines)+'\n')

    figures = root/'analysis_figures'
    figures.mkdir(exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    for ax, metric, scale, ylabel in zip(axes, ['coverage_percent', 'post_warmup_mean_age_s'],
                                       [1, 60], ['Unique coverage [%]', 'Post-startup mean observation age [min]']):
        means = [summaries[m][metric]['mean']/scale for m in METHODS]
        errors = [summaries[m][metric]['sd']/scale for m in METHODS]
        ax.bar(range(len(METHODS)), means, yerr=errors, capsize=3, color=['#d66d54' if m == 'full' else '#548c9b' for m in METHODS])
        ax.set_xticks(range(len(METHODS)), METHODS, rotation=35, ha='right')
        ax.set_ylabel(ylabel)
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('Identical physical constraints · 10 seeds · error bars = sample SD')
    fig.tight_layout()
    fig.savefig(figures/'physical_comparison.png', dpi=180)
    plt.close(fig)
    print(f'Complete: {len(rows)} prespecified runs; report at {root / "report.md"}')


if __name__ == '__main__':
    main()
