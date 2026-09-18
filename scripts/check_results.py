"""Independently check stored mission artifacts and optional deterministic rerun.

Usage: python scripts/check_results.py results/tamu_v1
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def check_age_and_sampled_cells(arr, cfg, metric):
    """Independent inverse-camera checks: 96 fixed sampled AOI cells per run.

    Does not call the production rasterizer or measurement code. Full-field
    aggregates are checked separately; this is not a full geometry re-evaluation.
    """
    mask = arr['aoi_mask']
    chosen = np.random.default_rng(17092026).choice(np.flatnonzero(mask), 96, replace=False)
    row, col = np.unravel_index(chosen, mask.shape)
    ticks = np.arange(0, cfg['horizon_s'], cfg['observation_step_s'])
    exposure = np.zeros((len(ticks), len(chosen)), dtype=np.int16)
    positions, states = arr['positions'], arr['states']
    for u in range(cfg['uavs']):
        pose = positions[ticks, u]
        dx = arr['grid_x'][col][None, :]-pose[:, 0, None]
        dy = arr['grid_y'][row][None, :]-pose[:, 1, None]
        c, s = np.cos(pose[:, 2, None]), np.sin(pose[:, 2, None])
        inside = (np.abs(dx*c+dy*s) <= cfg['fov_forward_m']/2+1e-8)
        inside &= np.abs(-dx*s+dy*c) <= cfg['fov_cross_m']/2+1e-8
        exposure += inside & (states[ticks, u, None] == 2)
    observed = exposure > 0
    entering = observed & ~np.vstack([np.zeros((1, len(chosen)), bool), observed[:-1]])
    np.testing.assert_array_equal(exposure.sum(axis=0), arr['coverage_counts'].ravel()[chosen])
    np.testing.assert_array_equal(entering.sum(axis=0), arr['coverage_episodes'].ravel()[chosen])
    for j, cell in enumerate(chosen):
        visits = ticks[observed[:, j]]
        endpoints = np.concatenate(([0], visits, [cfg['horizon_s']]))
        assert arr['terminal_age_s'].ravel()[cell] == endpoints[-1]-endpoints[-2]
        assert arr['max_blind_time_s'].ravel()[cell] == np.diff(endpoints).max()
    age = arr['mean_age_curve']
    np.testing.assert_allclose(age[:, 1].mean(), metric['mean_observation_age_s'])
    np.testing.assert_allclose(age[age[:, 0] >= cfg['warmup_s'], 1].mean(), metric['post_warmup_mean_age_s'])
    np.testing.assert_allclose(np.percentile(arr['terminal_age_s'][mask], 95), metric['terminal_p95_age_s'])
    np.testing.assert_allclose(arr['terminal_age_s'][mask].mean(), age[-1, 1]+cfg['observation_step_s'])
    assert arr['blind_interval_histogram'].sum() == metric['completed_blind_intervals']
    for threshold in (80, 95):
        curve = arr['coverage_curve']
        reached = curve[curve[:, 1] >= threshold*metric['aoi_cells']/100]
        expected = int(reached[0, 0]) if len(reached) else None
        assert metric[f'time_to_{threshold}_percent_s'] == expected


def check_run(path):
    data = json.loads((path/'result.json').read_text())
    assert hashlib.sha256((path/'simulation.npz').read_bytes()).hexdigest() == data['provenance']['artifact_sha256']
    with np.load(path/'simulation.npz') as archive:
        arr = {key: archive[key] for key in archive.files}
    cfg, metric = data['config'], data['metrics']
    assert np.count_nonzero(arr['aoi_mask']) == metric['aoi_cells']
    assert np.count_nonzero(arr['coverage_counts']) == metric['covered_cells']
    assert int(arr['coverage_counts'].sum()) == metric['cell_observations']
    np.testing.assert_array_equal(arr['coverage_counts'] > 0, arr['coverage_episodes'] > 0)
    assert np.all(np.diff(arr['coverage_curve'][:, 1]) >= 0)
    assert arr['coverage_curve'][-1, 1] == metric['covered_cells']
    assert np.max(np.sum(arr['states'] == 1, axis=1)) <= cfg['pads']
    assert np.all(arr['pads'][arr['states'] != 1] == -1)
    assert np.all(arr['pads'][arr['states'] == 1] >= 0)
    assert arr['battery'].min() >= 0 and arr['battery'].max() <= 100
    for t, state in enumerate(arr['states']):
        occupied = arr['pads'][t][arr['pads'][t] >= 0]
        assert len(occupied) == len(np.unique(occupied))
        grounded = state != 2
        np.testing.assert_allclose(arr['positions'][t, grounded, :2],
                                   np.broadcast_to(arr['ugv'][t], (grounded.sum(), 2)), atol=1e-5)
    for i, event in enumerate(data['sorties']):
        trajectory = arr[f'sortie_{i}']
        np.testing.assert_allclose(trajectory[0, :2], event['launch_point_m'], atol=1e-5)
        np.testing.assert_allclose(trajectory[-1, :2], event['rendezvous_point_m'], atol=1e-5)
        assert len(trajectory) == cfg['flight_s']+1
        start, end, u = event['takeoff_s'], min(event['landing_s'], cfg['horizon_s']), event['uav_id']
        assert np.all(arr['states'][start:end, u] == 2)
        assert np.all(np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1) <= cfg['uav_speed_mps']+1e-6)
        if cfg.get('physical_heading_limit'):
            turn = (np.diff(trajectory[:, 2])+np.pi) % (2*np.pi)-np.pi
            assert np.max(np.abs(turn)) <= np.deg2rad(cfg['heading_step_deg'])+1e-6
            # A constant-speed, curvature-bounded one-second path cannot hover:
            # its chord is at least that of a maximum-curvature circular arc.
            min_chord = cfg['uav_speed_mps']*np.sinc(np.deg2rad(cfg['heading_step_deg'])/(2*np.pi))
            assert np.min(np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1)) >= min_chord-1e-5
        np.testing.assert_allclose(arr['positions'][start:end, u], trajectory[:end-start], atol=1e-8)
        # Initial full charge and observed drain match configured flight time.
        assert abs(arr['battery'][start, u]-100) < 1e-6
        expected = 100*(1-np.arange(end-start+1)/cfg['flight_s'])
        np.testing.assert_allclose(arr['battery'][start:end+1, u], expected, atol=1e-5)
    if cfg.get('physical_heading_limit'):
        check_age_and_sampled_cells(arr, cfg, metric)
    return {'run': path.name, 'source_sha256': data['provenance']['source_sha256'],
            'independently_checked_cells': 96 if cfg.get('physical_heading_limit') else 0,
            'coverage_percent': metric['coverage_percent'], 'passed': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    checked = [check_run(path.parent) for path in sorted(args.root.glob('*/result.json'))]
    if not checked:
        raise RuntimeError('No runs found')
    hashes = set(row['source_sha256'] for row in checked)
    assert len(hashes) == 1, 'Mixed implementation versions in comparison'
    report = {'runs_checked': len(checked), 'common_source_sha256': hashes.pop(), 'runs': checked}
    (args.root/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(f'Validated {len(checked)} saved runs: endpoints, energy, pads, turn limits, metrics, hashes; v2 sampled-cell geometry and age.')


if __name__ == '__main__':
    main()
