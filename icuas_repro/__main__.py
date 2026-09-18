"""python -m icuas_repro run|compare|render"""
import argparse
from pathlib import Path

from .scenario import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    run = commands.add_parser('run')
    run.add_argument('--scenario', default=str(ROOT/'scenarios/tamu.json'))
    run.add_argument('--output', required=True)
    run.add_argument('--method', default='full', choices=['point', 'naive', 'full', 'no_heading', 'no_reinit', 'neither', 'sweep', 'greedy'])
    run.add_argument('--seed', type=int, default=0)
    run.add_argument('--horizon', type=int)
    run.add_argument('--planning-grid', type=float)
    compare = commands.add_parser('compare')
    compare.add_argument('--root', required=True)
    compare.add_argument('--scenario', default=str(ROOT/'scenarios/tamu.json'))
    compare.add_argument('--seeds', nargs='+', type=int, default=[0, 1, 2])
    compare.add_argument('--methods', nargs='+', default=['point', 'naive', 'full', 'no_heading', 'no_reinit', 'neither'])
    render = commands.add_parser('render')
    render.add_argument('runs', nargs='+')
    render.add_argument('--output', required=True)
    render.add_argument('--video', action='store_true')
    render.add_argument('--frames', type=int, default=240)
    args = parser.parse_args()
    if args.command == 'run':
        from .experiment import run
        overrides = {}
        if args.horizon:
            overrides['horizon_s'] = args.horizon
        if args.planning_grid:
            overrides['planning_grid_m'] = args.planning_grid
        run(args.output, args.scenario, args.method, args.seed, overrides)
    elif args.command == 'compare':
        from .experiment import run
        from .render import summary_table
        for seed in args.seeds:
            for method in args.methods:
                output = Path(args.root)/f'{method}_seed{seed}'
                if not (output/'result.json').exists():
                    run(output, args.scenario, method, seed)
        summary_table(args.root)
    else:
        from .render import render
        render(args.runs, args.output, args.video, args.frames)


if __name__ == '__main__':
    main()
