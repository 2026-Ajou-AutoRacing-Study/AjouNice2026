#!/usr/bin/env python3
"""Read-only candidate regression receipts on an offline probe, not deployment approval."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def qualify(replay, build, output, candidate):
    if output.exists():
        raise ValueError('Fresh qualification receipt required')
    report = json.loads((replay / 'report.json').read_text())
    builder = json.loads((build / 'build.json').read_text())
    with gzip.open(replay / 'frames.json.gz', 'rt') as f:
        frames = json.load(f)
    current_sources = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in builder['input_sha256']}
    if current_sources != builder['input_sha256']:
        raise ValueError('Production source no longer matches the linked snapshot')
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in report['source_sha256'].items()):
        raise ValueError('Frozen inputs or probe binaries changed')
    checks = dict(baseline_repeat_exact=report['baseline_repeat_exact'], disabled_probe_exact=report['disabled_probe_exact'])
    for case in report['cases']:
        old = frames[case + '/baseline']; repeated = frames[case + '/baseline_repeat']
        new = frames[case + '/' + candidate]
        if len(old['frames']) != len(new['frames']) or [f['stamp'] for f in old['frames']] != [f['stamp'] for f in new['frames']]:
            raise ValueError('Candidate input history or sensor stamps differ')
        checks[case + '/original_repeat_exact'] = old == repeated
        checks[case + '/confirmed_nonvehicle_exact'] = all(
            [t for t in a['tracks'] if t['confirmed'] and t['core_class'] not in (1, 2)] ==
            [t for t in b['tracks'] if t['confirmed'] and t['core_class'] not in (1, 2)]
            for a, b in zip(old['frames'], new['frames']))
        if case.startswith(('O02', 'P01')):
            checks[case + '/all_track_states_exact'] = old['frames'] == new['frames']
    for label in ('vehicle', 'pedestrian', 'obstacle'):
        old = report['pooled']['baseline'][label]; new = report['pooled'][candidate][label]
        checks[label + '/near_counts_exact'] = old['0_30m'] == new['0_30m']
        for band in old:
            checks[label + '/' + band + '/no_extra_unmatched_gate2m'] = (
                new[band]['unmatched_predictions_by_center_gate']['2.0'] <=
                old[band]['unmatched_predictions_by_center_gate']['2.0'])
    receipt = dict(candidate=candidate, diagnostic_checks=checks, diagnostic_checks_pass=all(checks.values()),
                   production_source_unchanged=True, production_enabled=False,
                   full_stack_or_new_bag_qualified=False,
                   source_sha256={str(replay / 'report.json'): hashlib.sha256((replay / 'report.json').read_bytes()).hexdigest(),
                                  str(replay / 'frames.json.gz'): hashlib.sha256((replay / 'frames.json.gz').read_bytes()).hexdigest()},
                   limitations=['Known development bags used to refine this candidate; no untouched test claim.',
                                'Core/source regression checks are not ROS/live timing/planning safety qualification.'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))
    return receipt


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('replay', 'build', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--candidate', default='weak010_moving_class090_xy')
    a = p.parse_args(); result = qualify(a.replay, a.build, a.output, a.candidate)
    raise SystemExit(0 if result['diagnostic_checks_pass'] else 1)
