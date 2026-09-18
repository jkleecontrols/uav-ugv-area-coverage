"""Execute the frozen v2 matrix, retaining every condition and source snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from icuas_repro.scenario import ROOT
from icuas_repro.experiment import run

METHODS = ['point', 'naive', 'sweep', 'greedy', 'full', 'no_heading', 'no_reinit', 'neither']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT/'results/tamu_physical_v2')
    parser.add_argument('--seeds', type=int, nargs='+', default=list(range(10)))
    parser.add_argument('--phase', choices=['main', 'sensitivity', 'grid'], default='main')
    args = parser.parse_args()
    root = args.root
    root.mkdir(parents=True, exist_ok=True)
    source = hashlib.sha256()
    for file in sorted((ROOT/'icuas_repro').glob('*.py')):
        source.update(file.name.encode())
        source.update(file.read_bytes())
    protocol = ROOT/'docs/EXPERIMENT_PROTOCOL_V2.md'
    manifest = {'source_sha256': source.hexdigest(),
                'protocol_sha256': hashlib.sha256(protocol.read_bytes()).hexdigest(),
                'main_seeds': list(range(10)), 'methods': METHODS,
                'heading_sensitivity_deg_s': [2.5, 5., 10.], 'sensitivity_seeds': [0, 1, 2]}
    existing = root/'manifest.json'
    if existing.exists():
        if json.loads(existing.read_text()) != manifest:
            raise RuntimeError('Source or protocol changed; use a new root, not a mixed batch')
    else:
        existing.write_text(json.dumps(manifest, indent=2)+'\n')
        snapshot = root/'source_snapshot'
        snapshot.mkdir()
        for file in (ROOT/'icuas_repro').glob('*.py'):
            shutil.copyfile(file, snapshot/file.name)
        shutil.copyfile(protocol, root/protocol.name)
    if args.phase == 'main':
        cases = [('main', method, seed, {}) for seed in args.seeds for method in METHODS]
    elif args.phase == 'sensitivity':
        cases = [(f'turn_{turn:g}', method, seed, {'heading_step_deg': turn})
                 for turn in [2.5, 10.] for seed in args.seeds if seed < 3
                 for method in ['full', 'greedy', 'naive']]
    else:
        cases = [('planning_1m', method, 0, {'planning_grid_m': 1.}) for method in ['full', 'greedy', 'naive']]
    for condition, method, seed, overrides in cases:
        output = root/condition/f'{method}_seed{seed}'
        if (output/'result.json').exists():
            data = json.loads((output/'result.json').read_text())
            if data['provenance']['source_sha256'] != manifest['source_sha256']:
                raise RuntimeError('Stale run in output directory')
            continue
        print(f'CASE {condition}/{method}/seed{seed}', flush=True)
        run(output, ROOT/'scenarios/tamu_physical_v2.json', method, seed, overrides)


if __name__ == '__main__':
    main()
