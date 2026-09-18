"""Publication plots and synchronized MP4, driven only by saved run artifacts."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon
import numpy as np

LABELS = {'point': 'Point planner', 'naive': 'Naive area', 'full': 'FOV + reinitialization',
          'sweep': 'Parallel-strip survey', 'greedy': 'Feasible area greedy',
          'no_heading': 'No heading constraint', 'no_reinit': 'No reinitialization',
          'neither': 'Neither component'}
COLORS = ['#e76f51', '#2a9d8f', '#6750a4']


def summary_table(root):
    root = Path(root)
    rows = []
    for file in sorted(root.glob('*/result.json')):
        data = json.loads(file.read_text())
        rows.append({'method': data['method'], 'seed': data['seed'], **data['metrics']})
    if not rows:
        raise ValueError('No run results found')
    with (root/'metrics.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    columns = ['coverage_percent', 'team_score_unique_weighted_cells',
               'revisited_covered_cells_percent', 'exposure_redundancy_percent',
               'mean_landing_to_charge_delay_s', 'runtime_s']
    summaries = []
    lines = ['# Measured ICUAS reimplementation results', '',
             '68-minute mission including startup; 1 m evaluation grid; 1 s exposure sampling.', '',
             '| Method | n | Coverage % (mean ± SD) | Revisited covered cells % | Delay s |',
             '| --- | ---: | ---: | ---: | ---: |']
    for method in dict.fromkeys(row['method'] for row in rows):
        subset = [row for row in rows if row['method'] == method]
        item = {'method': method, 'seeds': [row['seed'] for row in subset], 'n': len(subset)}
        for column in columns:
            values = np.asarray([row[column] for row in subset])
            item[column] = {'mean': float(values.mean()),
                            'sample_sd': float(values.std(ddof=1)) if len(values)>1 else None}
        summaries.append(item)
        c = item['coverage_percent']
        sd = f'{c["sample_sd"]:.2f}' if c['sample_sd'] is not None else 'n/a'
        lines.append(f'| {LABELS[method]} | {len(subset)} | {c["mean"]:.2f} ± {sd} | '
                     f'{item["revisited_covered_cells_percent"]["mean"]:.2f} | '
                     f'{item["mean_landing_to_charge_delay_s"]["mean"]:.1f} |')
    lines += ['', 'The manuscript tables are unverified and are not targets. These are new measurements.',
              'Revisit percentage counts cells covered in at least two distinct team-coverage episodes.',
              'Exposure redundancy includes adjacent camera exposures and is sampling-rate dependent.',
              'A deterministic, conflict-free schedule gives zero charging queue delay for every method.',
              'The physical camera and evaluation denominator are identical across all methods.']
    (root/'summary.json').write_text(json.dumps(summaries, indent=2)+'\n')
    (root/'summary.md').write_text('\n'.join(lines)+'\n')


def setup_map(ax, data):
    boundary = np.asarray(data['aoi_polygon_m'])
    ax.fill(boundary[:, 0], boundary[:, 1], color='#f2f3ef', zorder=0)
    ax.plot(boundary[:, 0], boundary[:, 1], color='#455a64', lw=.8)
    for road in data['road_polylines_m']:
        road = np.asarray(road)
        ax.plot(road[:, 0], road[:, 1], color='#c5c9cc', lw=1, zorder=1)
    route = np.asarray(data['map']['route_vertices_m'])
    ax.plot(route[:, 0], route[:, 1], color='#333c43', lw=1.4, linestyle='--', zorder=2)
    pad = 120
    ax.set_xlim(boundary[:, 0].min()-pad, boundary[:, 0].max()+pad)
    ax.set_ylim(boundary[:, 1].min()-pad, boundary[:, 1].max()+pad)
    ax.set_aspect('equal')
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.spines[['top', 'right']].set_visible(False)
    ax.set_title(LABELS[data['method']], fontweight='bold')


def render(runs, output, video=False, frames=240):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    payloads = [json.loads((Path(run)/'result.json').read_text()) for run in runs]
    arrays = [np.load(Path(run)/'simulation.npz') for run in runs]
    n = len(runs)
    plt.rcParams.update({'font.size': 10, 'font.family': 'DejaVu Sans'})
    fig, axes = plt.subplots(1, n, figsize=(6*n, 6), squeeze=False)
    for ax, data, arr in zip(axes[0], payloads, arrays):
        setup_map(ax, data)
        for i, event in enumerate(data['sorties']):
            trajectory = arr[f'sortie_{i}']
            end = min(len(trajectory), data['config']['horizon_s']-event['takeoff_s']+1)
            ax.plot(trajectory[:end, 0], trajectory[:end, 1], color=COLORS[event['uav_id']], alpha=.65, lw=.65)
        metric = data['metrics']
        ax.set_title(f'{LABELS[data["method"]]}\ncoverage {metric["coverage_percent"]:.2f}%', fontweight='bold')
    fig.suptitle('Measured UAV trajectories · TAMU road-constrained mission', fontsize=15)
    fig.tight_layout()
    fig.savefig(output/'trajectories.png', dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(1, n, figsize=(6*n, 6), squeeze=False)
    for ax, data, arr in zip(axes[0], payloads, arrays):
        setup_map(ax, data)
        field = np.ma.masked_where(~arr['aoi_mask'], np.minimum(arr['coverage_episodes'], 5))
        extent = (arr['grid_x'][0]-.5, arr['grid_x'][-1]+.5, arr['grid_y'][0]-.5, arr['grid_y'][-1]+.5)
        plot = ax.imshow(field, origin='lower', extent=extent, cmap='YlGnBu', vmin=0, vmax=5, alpha=.85)
        fig.colorbar(plot, ax=ax, shrink=.65, label='Coverage episodes (5 = 5 or more)')
    fig.tight_layout()
    fig.savefig(output/'coverage.png', dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for data, arr in zip(payloads, arrays):
        curve = arr['coverage_curve']
        ax.plot(curve[:, 0]/60, 100*curve[:, 1]/data['metrics']['aoi_cells'], label=LABELS[data['method']])
    ax.set(xlabel='Mission time [min]', ylabel='Unique area coverage [%]', ylim=(0, 100))
    ax.legend(loc='lower right')
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(output/'coverage_over_time.png', dpi=180)
    plt.close(fig)

    if all('mean_age_curve' in arr for arr in arrays):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for data, arr in zip(payloads, arrays):
            curve = arr['mean_age_curve']
            ax.plot(curve[:, 0]/60, curve[:, 1]/60, label=LABELS[data['method']])
        ax.set(xlabel='Mission time [min]', ylabel='Mean observation age [min]')
        ax.axvline(payloads[0]['config'].get('warmup_s', 0)/60, color='gray', ls=':', label='End of startup window')
        ax.legend()
        ax.grid(alpha=.2)
        fig.tight_layout()
        fig.savefig(output/'observation_age.png', dpi=180)
        plt.close(fig)

    if not video:
        return
    fig, axes = plt.subplots(1, n, figsize=(6*n, 6.2), squeeze=False)
    end = min(data['config']['horizon_s'] for data in payloads)
    times = np.linspace(0, end-1, frames).astype(int)
    artists = []
    previews = []
    for ax, data in zip(axes[0], payloads):
        setup_map(ax, data)
        # Include airspace used for the feasible return/loiter trajectories.
        index = len(artists)
        if data['config'].get('physical_heading_limit', False):
            from shapely.geometry import Polygon as ShapelyPolygon
            from .raster import Raster
            raster = Raster(ShapelyPolygon(data['aoi_polygon_m']), 10.,
                            data['config']['fov_forward_m'], data['config']['fov_cross_m'])
            seen = np.zeros(raster.size, dtype=np.uint8)
            history = np.zeros((len(times), *raster.shape), dtype=np.uint8)
            last = -1
            for frame, t in enumerate(times):
                for tick in range(last+1, t+1):
                    for u in range(data['config']['uavs']):
                        if arrays[index]['states'][tick, u] == 2:
                            seen[raster.footprint(arrays[index]['positions'][tick, u])] = 1
                history[frame] = seen.reshape(raster.shape)
                last = t
            extent = (raster.x[0]-5, raster.x[-1]+5, raster.y[0]-5, raster.y[-1]+5)
            from matplotlib.colors import ListedColormap
            preview = ax.imshow(history[0], origin='lower', extent=extent, vmin=0, vmax=1,
                                cmap=ListedColormap(['#ffffff00', '#86c9e766']), zorder=1)
            previews.append((preview, history))
        else:
            previews.append(None)
        xy = arrays[index]['positions'][:, :, :2].reshape(-1, 2)
        ax.set_xlim(min(ax.get_xlim()[0], xy[:, 0].min()-60), max(ax.get_xlim()[1], xy[:, 0].max()+60))
        ax.set_ylim(min(ax.get_ylim()[0], xy[:, 1].min()-60), max(ax.get_ylim()[1], xy[:, 1].max()+60))
        ugv, = ax.plot([], [], 's', color='#202c39', markersize=7, zorder=8)
        drones, trails, footprints = [], [], []
        for u in range(data['config']['uavs']):
            drone, = ax.plot([], [], 'o', color=COLORS[u], markersize=5, zorder=10)
            trail, = ax.plot([], [], color=COLORS[u], alpha=.65, lw=1, zorder=5)
            footprint = Polygon(np.zeros((4, 2)), color=COLORS[u], alpha=.22, zorder=4)
            ax.add_patch(footprint)
            drones.append(drone)
            trails.append(trail)
            footprints.append(footprint)
        status = ax.text(.01, -.17, '', transform=ax.transAxes, fontsize=9, family='monospace')
        artists.append((ugv, drones, trails, footprints, status))
    title = fig.suptitle('', fontsize=14, y=.98)
    fig.subplots_adjust(left=.04, right=.98, top=.88, bottom=.20, wspace=.25)
    if any(preview is not None for preview in previews):
        fig.text(.5, .025, 'Blue shading: observed area (10 m display grid; reported metrics: 1 m).  F: flying  C: charging  W: waiting.',
                 ha='center', fontsize=9)
    def update(frame):
        t = int(times[frame])
        title.set_text(f'TAMU UAV–UGV surveillance  |  mission {t//60:02d}:{t%60:02d}  |  identical camera & charging schedule')
        for data, arr, group, preview in zip(payloads, arrays, artists, previews):
            if preview is not None:
                preview[0].set_data(preview[1][frame])
            ugv, drones, trails, footprints, status = group
            ugv.set_data([arr['ugv'][t, 0]], [arr['ugv'][t, 1]])
            labels = []
            for u, (drone, trail, patch) in enumerate(zip(drones, trails, footprints)):
                x, y, h = arr['positions'][t, u]
                drone.set_data([x], [y])
                mask = arr['states'][max(0, t-180):t+1, u] == 2
                xy = arr['positions'][max(0, t-180):t+1, u, :2].copy()
                xy[~mask] = np.nan
                trail.set_data(xy[:, 0], xy[:, 1])
                f, c = data['config']['fov_forward_m']/2, data['config']['fov_cross_m']/2
                local = np.array([[f, c], [f, -c], [-f, -c], [-f, c]])
                rotation = np.array([[np.cos(h), -np.sin(h)], [np.sin(h), np.cos(h)]])
                patch.set_xy(local @ rotation.T+[x, y])
                patch.set_visible(arr['states'][t, u] == 2)
                state = ('WAIT', 'CHARGE', 'FLY')[arr['states'][t, u]]
                labels.append(f'U{u+1} {state[0]} {arr["battery"][t, u]:3.0f}%')
            curve = arr['coverage_curve']
            idx = min(len(curve)-1, t//data['config']['observation_step_s'])
            coverage = 100*curve[idx, 1]/data['metrics']['aoi_cells']
            age = f'  age {arr["mean_age_curve"][t, 1]/60:.1f}m' if 'mean_age_curve' in arr else ''
            status.set_text(' | '.join(labels)+'\n'+f'Coverage {coverage:5.1f}%{age}  pads {np.sum(arr["states"][t] == 1)}/{data["config"]["pads"]}')
    animation = FuncAnimation(fig, update, frames=len(times), blit=False)
    update(len(times)//2)
    fig.savefig(output/'video_snapshot.png', dpi=120)
    writer = FFMpegWriter(fps=12, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '20'])
    animation.save(str(output/'comparison.mp4'), writer=writer, dpi=100)
    plt.close(fig)
    print(f'Video saved: {output / "comparison.mp4"}', flush=True)
