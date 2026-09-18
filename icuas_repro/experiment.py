"""Run, validate and save actual simulation artifacts."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np

from .planner import plan_team, wrap
from .raster import Raster
from .scenario import load_scenario, ROOT
from .scheduling import solve_max_flight_schedule
from .simulation import sortie_events

METHODS = ('point', 'naive', 'full', 'no_heading', 'no_reinit', 'neither', 'sweep', 'greedy')


def timeline(schedule, config):
    horizon = config['horizon_s']
    states = np.zeros((horizon+1, config['uavs']), dtype=np.int8)
    battery = np.zeros(states.shape, dtype=float)
    pads = np.full(states.shape, -1, dtype=np.int8)
    intervals = []
    for u in schedule.selected_uavs:
        first = schedule.charge_start_slot[u]*schedule.slot_seconds
        charge = schedule.charge_slots[u]*schedule.slot_seconds
        flight = schedule.flight_slots[u]*schedule.slot_seconds
        cycle = charge+flight
        for t in range(first, horizon+1):
            phase = (t-first) % cycle
            states[t, u] = 1 if phase < charge else 2
            battery[t, u] = 100*phase/charge if phase < charge else 100*(1-(phase-charge)/flight)
        for start in range(first, horizon+1, cycle):
            intervals.append({'uav_id': u, 'start_s': start, 'end_s': start+charge})
    free = np.zeros(config['pads'], dtype=int)
    for interval in sorted(intervals, key=lambda item: (item['start_s'], item['uav_id'])):
        available = np.where(free <= interval['start_s'])[0]
        if not len(available):
            raise RuntimeError('Charging pad overlap in the realized schedule')
        pad = int(available[0])
        interval['pad_id'] = pad
        free[pad] = interval['end_s']
        pads[interval['start_s']:min(horizon+1, interval['end_s']), interval['uav_id']] = pad
    return states, battery, pads, intervals


def validate(events, trajectories, states, battery, pads, config, route, method):
    endpoint_error, max_speed, max_turn = 0., 0., 0.
    for event, trajectory in zip(events, trajectories):
        expected = event.landing_s-event.takeoff_s+1
        if trajectory.shape != (expected, 3) or not np.isfinite(trajectory).all():
            raise AssertionError('Invalid trajectory samples')
        error = max(np.linalg.norm(trajectory[0, :2]-route.position_at_time(event.takeoff_s, config['ugv_speed_mps'])),
                    np.linalg.norm(trajectory[-1, :2]-route.position_at_time(event.landing_s, config['ugv_speed_mps'])))
        endpoint_error = max(endpoint_error, error)
        max_speed = max(max_speed, float(np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1).max()))
        max_turn = max(max_turn, float(np.max(np.abs(wrap(np.diff(trajectory[:, 2]))))))
    if endpoint_error > 1e-4 or max_speed > config['uav_speed_mps']+1e-5:
        raise AssertionError(f'Speed or rendezvous constraint violated: speed={max_speed}, error={endpoint_error}')
    if (config.get('physical_heading_limit', False) or method in ('full', 'point', 'no_reinit', 'greedy', 'sweep')) and max_turn > np.deg2rad(config['heading_step_deg'])+1e-6:
        raise AssertionError('Heading limit violated')
    if battery.min() < -1e-8 or battery.max() > 100+1e-8:
        raise AssertionError('Battery constraint violated')
    if np.any(np.sum(states == 1, axis=1) > config['pads']):
        raise AssertionError('Pad capacity exceeded')
    for row in pads:
        allocated = row[row >= 0]
        if len(allocated) != len(set(allocated)):
            raise AssertionError('Pad assigned twice')
    return {'max_rendezvous_error_m': endpoint_error, 'max_sampled_speed_mps': max_speed,
            'max_heading_change_deg': float(np.rad2deg(max_turn)),
            'max_charging_occupancy': int(np.sum(states == 1, axis=1).max()),
            'all_constraints_passed': True}


def measure(raster, positions, states, config, diagnostics=None):
    """At each second, union simultaneous footprints before counting entries.

    Exposure redundancy depends on the observation rate. Revisit redundancy
    instead counts new team-coverage episodes after a cell was absent for at
    least one sampled second. Both definitions are reported, not conflated.
    """
    counts = np.zeros(raster.size, dtype=np.int32)
    episodes = np.zeros(raster.size, dtype=np.uint16)
    last_seen = np.full(raster.size, -2, dtype=np.int32)
    curve = []
    seen_cells = 0
    valid = int(raster.valid.sum())
    warmup = config.get('warmup_s', 0)
    sum_last_observed = 0
    age_curve = []
    max_blind = np.zeros(raster.size, dtype=np.int32)
    blind_histogram = np.zeros(config['horizon_s']+1, dtype=np.int64)
    window_seen = np.zeros(raster.size, dtype=bool)
    for t in range(0, config['horizon_s'], config['observation_step_s']):
        ids_list = []
        for u in range(config['uavs']):
            if states[t, u] == 2:
                ids = raster.footprint(positions[t, u])
                counts[ids] += 1
                ids_list.append(ids)
        if ids_list:
            ids = np.unique(np.concatenate(ids_list))
            seen_cells += int(np.count_nonzero(last_seen[ids] == -2))
            entering = last_seen[ids] < t-config['observation_step_s']
            age_before_reset = t-np.maximum(last_seen[ids], 0)
            sum_last_observed += int(age_before_reset.sum())
            max_blind[ids] = np.maximum(max_blind[ids], age_before_reset)
            if t >= warmup:
                completed = ids[entering & (last_seen[ids] >= 0)]
                gaps = t-last_seen[completed]
                blind_histogram += np.bincount(gaps, minlength=len(blind_histogram))
                window_seen[ids] = True
            episodes[ids[entering]] += 1
            last_seen[ids] = t
        curve.append((t, seen_cells))
        age_curve.append((t, t-sum_last_observed/valid))
    covered = int(np.count_nonzero(counts))
    total = int(counts.sum())
    age_curve = np.asarray(age_curve)
    terminal_age = config['horizon_s']-np.maximum(last_seen, 0)
    max_blind = np.maximum(max_blind, terminal_age)
    completed_count = int(blind_histogram.sum())
    cumulative = np.cumsum(blind_histogram)
    window = age_curve[:, 0] >= warmup
    metrics = {'coverage_percent': 100*covered/valid,
               'team_score_unique_weighted_cells': covered,
               'covered_area_m2': covered*raster.resolution**2,
               'aoi_cells': valid, 'covered_cells': covered,
               'cell_observations': total,
               'exposure_redundancy_percent': 100*(total-covered)/total if total else 0.,
               'revisited_covered_cells_percent': 100*np.count_nonzero(episodes > 1)/covered if covered else 0.,
               'coverage_episodes': int(episodes.sum()),
               'observation_step_s': config['observation_step_s'],
               'evaluation_grid_m': raster.resolution}
    metrics.update({
        'mean_observation_age_s': float(age_curve[:, 1].mean()),
        'post_warmup_mean_age_s': float(age_curve[window, 1].mean()) if window.any() else None,
        'terminal_p95_age_s': float(np.percentile(terminal_age[raster.valid], 95)),
        'max_censored_unobserved_time_s': int(max_blind[raster.valid].max()),
        'mean_max_censored_unobserved_time_s': float(max_blind[raster.valid].mean()),
        'post_warmup_coverage_percent': 100*np.count_nonzero(window_seen)/valid,
        'completed_blind_intervals': completed_count,
        'completed_blind_interval_mean_s': float(np.dot(np.arange(len(blind_histogram)), blind_histogram)/completed_count) if completed_count else None,
        'completed_blind_interval_p95_s': int(np.searchsorted(cumulative, .95*completed_count)) if completed_count else None,
    })
    coverage_curve = np.asarray(curve)
    for threshold in (80, 95):
        reached = coverage_curve[coverage_curve[:, 1] >= threshold*valid/100]
        metrics[f'time_to_{threshold}_percent_s'] = int(reached[0, 0]) if len(reached) else None
    if diagnostics is not None:
        diagnostics.update({'mean_age_curve': age_curve,
                            'terminal_age_s': terminal_age.reshape(raster.shape),
                            'max_blind_time_s': max_blind.reshape(raster.shape),
                            'blind_interval_histogram': blind_histogram})
    return metrics, counts, episodes, np.asarray(curve)


def run(output, scenario, method='full', seed=0, overrides=None):
    if method not in METHODS:
        raise ValueError(method)
    started = time.perf_counter()
    config, aoi, roads, route, metadata = load_scenario(scenario)
    config.update(overrides or {})
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if (output/'result.json').exists():
        raise FileExistsError(f'Use a fresh output directory: {output}')
    schedule = solve_max_flight_schedule([config['charge_s']]*config['uavs'],
                                         [config['flight_s']]*config['uavs'],
                                         config['pads'], config['slot_s'])
    # Warm start: vehicles initially uncharged on board, not airborne.
    # Include full trajectories of sorties truncated by the reporting horizon.
    events = [event for event in sortie_events(schedule, route, config['ugv_speed_mps'],
                                               config['horizon_s']+config['flight_s']+config['charge_s'])
              if event.takeoff_s < config['horizon_s']]
    planning = Raster(aoi, config['planning_grid_m'], config['fov_forward_m'], config['fov_cross_m'])
    print(f'{method} seed={seed}: planning {len(events)} sorties; AOI {aoi.area/1e6:.3f} km²', flush=True)
    planning_started = time.perf_counter()
    trajectories, search_log = plan_team(events, config, planning, seed, method)
    planning_runtime = time.perf_counter()-planning_started
    states, battery, pads, intervals = timeline(schedule, config)
    ugv = np.asarray([route.position_at_time(t, config['ugv_speed_mps']) for t in range(config['horizon_s']+1)])
    positions = np.zeros((len(ugv), config['uavs'], 3))
    positions[:, :, :2] = ugv[:, None, :]
    for event, trajectory in zip(events, trajectories):
        end = min(event.landing_s, config['horizon_s'])
        positions[event.takeoff_s:end+1, event.uav_id] = trajectory[:end-event.takeoff_s+1]
    checks = validate(events, trajectories, states, battery, pads, config, route, method)
    print(f'{method} seed={seed}: measuring on {config["evaluation_grid_m"]} m grid', flush=True)
    evaluation = Raster(aoi, config['evaluation_grid_m'], config['fov_forward_m'], config['fov_cross_m'])
    diagnostics = {}
    metrics, counts, episodes, curve = measure(evaluation, positions, states, config, diagnostics)
    metrics['planning_runtime_s'] = planning_runtime
    delays = []
    for event in events:
        if event.landing_s <= config['horizon_s']:
            starts = [entry['start_s'] for entry in intervals
                      if entry['uav_id'] == event.uav_id and entry['start_s'] >= event.landing_s]
            if not starts:
                raise AssertionError('Completed sortie has no charging event')
            delays.append(min(starts)-event.landing_s)
    metrics['landing_to_charge_delay_samples_s'] = delays
    metrics['mean_landing_to_charge_delay_s'] = float(np.mean(delays)) if delays else None
    metrics['airborne_vehicle_seconds'] = int(np.count_nonzero(states[:-1] == 2))
    metrics['runtime_s'] = time.perf_counter()-started
    arrays = {'positions': positions, 'ugv': ugv, 'states': states, 'battery': battery,
              'pads': pads, 'coverage_counts': counts.reshape(evaluation.shape),
              'coverage_episodes': episodes.reshape(evaluation.shape),
              'aoi_mask': evaluation.mask, 'grid_x': evaluation.x, 'grid_y': evaluation.y,
              'coverage_curve': curve}
    arrays.update({f'sortie_{i}': trajectory for i, trajectory in enumerate(trajectories)})
    arrays.update(diagnostics)
    np.savez_compressed(output/'simulation.npz', **arrays)
    fingerprint = hashlib.sha256()
    for file in sorted((ROOT/'icuas_repro').glob('*.py')):
        fingerprint.update(file.name.encode())
        fingerprint.update(file.read_bytes())
    payload = {'method': method, 'seed': seed, 'config': config, 'map': metadata,
               'aoi_polygon_m': list(aoi.exterior.coords),
               'road_polylines_m': [list(road.coords) for road in roads],
               'schedule': asdict(schedule), 'charge_intervals': intervals,
               'sorties': [event.as_dict() for event in events], 'search_log': search_log,
               'validation': checks, 'metrics': metrics,
               'provenance': {'python': platform.python_version(), 'numpy': np.__version__,
                              'source_sha256': fingerprint.hexdigest(),
                              'map_sha256': hashlib.sha256((ROOT/config['map_path']).read_bytes()).hexdigest(),
                              'artifact_sha256': hashlib.sha256((output/'simulation.npz').read_bytes()).hexdigest(),
                              'protocol_sha256': hashlib.sha256((ROOT/config['protocol']).read_bytes()).hexdigest() if 'protocol' in config else None,
                              'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()},
               'status': 'measured reimplementation; not reproduction of manuscript tables'}
    (output/'result.json').write_text(json.dumps(payload, indent=2)+'\n')
    print(json.dumps(metrics, indent=2), flush=True)
    return payload
