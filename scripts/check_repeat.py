"""Compare independently rerun saved artifacts, excluding measured runtimes."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('original', type=Path)
    parser.add_argument('repeat', type=Path)
    args = parser.parse_args()
    first = json.loads((args.original/'result.json').read_text())
    second = json.loads((args.repeat/'result.json').read_text())
    for field in ['config', 'method', 'seed', 'schedule', 'sorties', 'charge_intervals',
                  'search_log', 'validation']:
        assert first[field] == second[field], field
    excluded = ['runtime_s', 'planning_runtime_s']
    assert {k: v for k, v in first['metrics'].items() if k not in excluded} == {
        k: v for k, v in second['metrics'].items() if k not in excluded}
    assert first['provenance']['source_sha256'] == second['provenance']['source_sha256']
    with np.load(args.original/'simulation.npz') as a, np.load(args.repeat/'simulation.npz') as b:
        assert set(a.files) == set(b.files)
        for key in a.files:
            np.testing.assert_array_equal(a[key], b[key], err_msg=key)
        report = {'original': str(args.original), 'repeat': str(args.repeat),
                  'passed': True, 'arrays_identical': len(a.files),
                  'non_runtime_metrics_identical': True, 'excluded_metrics': excluded,
                  'source_sha256': first['provenance']['source_sha256']}
    (args.repeat/'repeat_verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
