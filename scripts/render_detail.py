"""Render a fixed saved mission: no planning, resimulation, or fabricated paths."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon, Patch
import numpy as np
from shapely.geometry import Polygon as ShapePolygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from icuas_repro.raster import Raster
from icuas_repro.render import setup_map, COLORS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=720)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = json.loads((args.run/'result.json').read_text())
    with np.load(args.run/'simulation.npz') as archive:
        arr = {key: archive[key] for key in archive.files}
    cfg = data['config']
    times = np.linspace(0, cfg['horizon_s']-1, args.frames).astype(int)
    raster = Raster(ShapePolygon(data['aoi_polygon_m']), 10,
                    cfg['fov_forward_m'], cfg['fov_cross_m'])
    last_seen = np.zeros(raster.size, dtype=np.int32)
    history = np.zeros((len(times), *raster.shape), dtype=np.uint16)
    tick = 0
    for frame, t in enumerate(times):
        while tick <= t:
            for u in range(cfg['uavs']):
                if arr['states'][tick, u] == 2:
                    last_seen[raster.footprint(arr['positions'][tick, u])] = tick
            tick += 1
        history[frame] = (t-last_seen).reshape(raster.shape)
    print('Observation-age display prepared from saved one-second poses.', flush=True)
    plt.rcParams.update({'font.size': 10, 'font.family': 'DejaVu Sans'})
    fig = plt.figure(figsize=(12, 8), facecolor='white')
    grid = fig.add_gridspec(4, 2, width_ratios=[1.35, 1],
                           left=.065, right=.96, top=.88, bottom=.18,
                           wspace=.35, hspace=.65)
    map_ax = fig.add_subplot(grid[:, 0])
    setup_map(map_ax, data)
    map_ax.set_title('Full method · seed 0', fontweight='bold')
    xy = arr['positions'][:, :, :2].reshape(-1, 2)
    map_ax.set_xlim(min(map_ax.get_xlim()[0], xy[:, 0].min()-60),
                    max(map_ax.get_xlim()[1], xy[:, 0].max()+60))
    map_ax.set_ylim(min(map_ax.get_ylim()[0], xy[:, 1].min()-60),
                    max(map_ax.get_ylim()[1], xy[:, 1].max()+60))
    extent = (raster.x[0]-5, raster.x[-1]+5, raster.y[0]-5, raster.y[-1]+5)
    field = map_ax.imshow(np.ma.masked_where(~raster.mask, history[0]/60),
                          origin='lower', extent=extent, cmap='YlOrRd',
                          vmin=0, vmax=cfg['horizon_s']/60, zorder=.5)
    color_ax = fig.add_axes([.095, .09, .34, .015])
    fig.colorbar(field, cax=color_ax, orientation='horizontal', label='Observation age [min]')
    ugv, = map_ax.plot([], [], 's', color='#202c39', ms=8, zorder=9, label='UGV')
    groups = []
    for u in range(cfg['uavs']):
        marker, = map_ax.plot([], [], 'o', color=COLORS[u], ms=5, zorder=10, label=f'UAV {u+1}')
        trail, = map_ax.plot([], [], color=COLORS[u], lw=1, zorder=5)
        patch = Polygon(np.zeros((4, 2)), facecolor=COLORS[u], edgecolor=COLORS[u], alpha=.4, zorder=6)
        map_ax.add_patch(patch)
        groups.append((marker, trail, patch))
    map_ax.legend(loc='upper left', fontsize=8, ncol=2)

    battery_ax = fig.add_subplot(grid[0, 1])
    bars = battery_ax.barh(np.arange(3), [0]*3, color=COLORS, height=.5)
    battery_ax.set(xlim=(0, 125), yticks=np.arange(3), yticklabels=['UAV 1', 'UAV 2', 'UAV 3'])
    battery_ax.set_xticks([0, 50, 100], ['0%', '50%', '100%'])
    battery_ax.invert_yaxis()
    battery_ax.set_title('Battery and charging pads', loc='left', fontsize=11)
    battery_labels = [battery_ax.text(102, u, '', va='center', fontsize=8) for u in range(3)]
    coverage_ax = fig.add_subplot(grid[1, 1])
    age_ax = fig.add_subplot(grid[2, 1])
    curve = arr['coverage_curve']
    coverage_values = 100*curve[:, 1]/data['metrics']['aoi_cells']
    age = arr['mean_age_curve']
    coverage_line, = coverage_ax.plot([], [], color='#217c8a', lw=2)
    age_line, = age_ax.plot([], [], color='#d46e30', lw=2)
    for ax in [coverage_ax, age_ax]:
        ax.set_xlim(0, cfg['horizon_s']/60)
        ax.grid(alpha=.2)
        ax.axvline(cfg['warmup_s']/60, color='gray', ls=':', lw=1)
    coverage_ax.set_ylim(0, 102)
    age_ax.set_ylim(0, max(35, age[:, 1].max()/60*1.08))
    coverage_ax.set_ylabel('Coverage [%]')
    age_ax.set_ylabel('Mean age [min]')
    coverage_value = coverage_ax.text(.98, .08, '', transform=coverage_ax.transAxes, ha='right')
    age_value = age_ax.text(.98, .08, '', transform=age_ax.transAxes, ha='right')
    schedule_ax = fig.add_subplot(grid[3, 1])
    state_colors = ['#dee2e6', '#eead4c', '#588dba']
    for u in range(3):
        state = arr['states'][:cfg['horizon_s'], u]
        starts = np.r_[0, np.flatnonzero(np.diff(state))+1]
        ends = np.r_[starts[1:], len(state)]
        for start, end in zip(starts, ends):
            schedule_ax.broken_barh([(start/60, (end-start)/60)], (u-.3, .6),
                                    facecolors=state_colors[state[start]])
    cursor = schedule_ax.axvline(0, color='black', lw=1.5)
    schedule_ax.set(xlim=(0, cfg['horizon_s']/60), ylim=(-.6, 2.6),
                    yticks=range(3), yticklabels=['U1', 'U2', 'U3'], xlabel='Mission time [min]')
    schedule_ax.invert_yaxis()
    schedule_ax.legend(handles=[Patch(color=c, label=s) for c, s in zip(state_colors, ['Wait', 'Charge', 'Fly'])],
                       loc='upper center', bbox_to_anchor=(.5, -.5), ncol=3, fontsize=8, frameon=False)
    title = fig.suptitle('', fontsize=15, y=.97)
    fig.text(.5, .925, '3 UAVs / 2 pads · 10 m/s · turn limit 5°/s · 20 min flight / 14 min charge',
             ha='center', fontsize=10)
    fig.text(.5, .018, 'Saved simulation only. Age map: 10 m display grid; curves: 1 m evaluation. Never-seen cells included; initial age = 0.',
             ha='center', fontsize=8)

    def update(frame):
        t = int(times[frame])
        title.set_text(f'ICUAS area-surveillance implementation  |  mission {t//60:02d}:{t%60:02d}')
        field.set_data(np.ma.masked_where(~raster.mask, history[frame]/60))
        ugv.set_data([arr['ugv'][t, 0]], [arr['ugv'][t, 1]])
        for u, (marker, trail, patch) in enumerate(groups):
            x, y, h = arr['positions'][t, u]
            marker.set_data([x], [y])
            start = max(0, t-180)
            path = arr['positions'][start:t+1, u, :2].copy()
            path[arr['states'][start:t+1, u] != 2] = np.nan
            trail.set_data(path[:, 0], path[:, 1])
            f, c = cfg['fov_forward_m']/2, cfg['fov_cross_m']/2
            local = np.array([[f, c], [f, -c], [-f, -c], [-f, c]])
            rotation = np.array([[np.cos(h), -np.sin(h)], [np.sin(h), np.cos(h)]])
            patch.set_xy(local @ rotation.T+[x, y])
            patch.set_visible(arr['states'][t, u] == 2)
            bars[u].set_width(arr['battery'][t, u])
            battery_labels[u].set_text(('WAIT', 'CHARGE', 'FLY')[arr['states'][t, u]])
        battery_ax.set_title(f'Battery · pads occupied {np.sum(arr["states"][t] == 1)}/2', loc='left', fontsize=11)
        idx = np.searchsorted(curve[:, 0], t, side='right')
        coverage_line.set_data(curve[:idx, 0]/60, coverage_values[:idx])
        age_line.set_data(age[:idx, 0]/60, age[:idx, 1]/60)
        coverage_value.set_text(f'{coverage_values[idx-1]:.2f}%')
        age_value.set_text(f'{age[idx-1, 1]/60:.2f} min')
        cursor.set_xdata([t/60, t/60])

    animation = FuncAnimation(fig, update, frames=len(times), blit=False)
    for fraction, name in [(0.5, 'detail_snapshot.png'), (1., 'detail_final.png')]:
        update(min(len(times)-1, int(fraction*len(times))))
        fig.savefig(args.output/name, dpi=120)
    writer = FFMpegWriter(fps=12, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '20'])
    animation.save(str(args.output/'full_detail.mp4'), writer=writer, dpi=120)
    plt.close(fig)
    (args.output/'provenance.json').write_text(json.dumps({
        'run': str(args.run), 'artifact_sha256': data['provenance']['artifact_sha256'],
        'frames': len(times), 'fps': 12, 'display_grid_m': 10,
        'metrics_grid_m': cfg['evaluation_grid_m'], 'seed': data['seed'],
        'simulation_time_s': [int(times[0]), int(times[-1])]}, indent=2)+'\n')
    print(f'Saved {args.output / "full_detail.mp4"}', flush=True)


if __name__ == '__main__':
    main()
