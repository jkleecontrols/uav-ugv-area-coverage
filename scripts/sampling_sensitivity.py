"""Estimate coverage sensitivity using half-second interpolated saved poses.

This is a numerical diagnostic, NOT a new planner run or exact 60 fps sensing.
Positions are linearly interpolated; headings follow the shortest angular arc.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from shapely.geometry import Polygon
from icuas_repro.raster import Raster
from icuas_repro.experiment import measure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    data = json.loads((args.run/'result.json').read_text())
    arr = np.load(args.run/'simulation.npz')
    original = arr['positions']
    positions = np.empty((2*(len(original)-1)+1, *original.shape[1:]))
    positions[::2] = original
    positions[1::2, :, :2] = (original[:-1, :, :2]+original[1:, :, :2])/2
    delta = (original[1:, :, 2]-original[:-1, :, 2]+np.pi) % (2*np.pi)-np.pi
    positions[1::2, :, 2] = original[:-1, :, 2]+delta/2
    states = np.repeat(arr['states'], 2, axis=0)[:-1]
    cfg = dict(data['config'])
    cfg['horizon_s'] *= 2  # evaluation ticks now correspond to half-seconds
    cfg['observation_step_s'] = 1
    raster = Raster(Polygon(data['aoi_polygon_m']), cfg['evaluation_grid_m'],
                    cfg['fov_forward_m'], cfg['fov_cross_m'])
    metric, _, _, _ = measure(raster, positions, states, cfg)
    report = {'pose_interpolation': 'linear xy, shortest-arc heading; approximate',
              'original_sample_s': 1., 'diagnostic_sample_s': .5,
              'original_coverage_percent': data['metrics']['coverage_percent'],
              'diagnostic_coverage_percent': metric['coverage_percent'],
              'difference_percentage_points': metric['coverage_percent']-data['metrics']['coverage_percent']}
    (args.run/'sampling_sensitivity.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
