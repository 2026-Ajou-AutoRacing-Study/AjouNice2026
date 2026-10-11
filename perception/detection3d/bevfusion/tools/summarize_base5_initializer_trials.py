#!/usr/bin/env python3
"""Read paired trial receipts; reject comparisons with other changed inputs."""
import argparse
import json
from pathlib import Path


def summarize(root):
    root = Path(root)
    manifest = json.loads((root/'run_manifest.json').read_text())
    status = json.loads((root/'initializer_trial_status.json').read_text())
    metrics = []
    for path in sorted(root.glob('*/vis_data/scalars.json')):
        for line in path.read_text().splitlines():
            row = json.loads(line)
            if 'val/center/mAP' not in row:
                continue
            metrics.append({key: value for key, value in row.items()
                            if key in ('step', 'epoch') or key.startswith('val/')})
    if status['state'] != 'finished' or not metrics:
        raise ValueError('Trial has not completed full validation: ' + str(root))
    expected_iterations = manifest['identity']['iterations_per_epoch'] * status['epoch']
    if status['iteration'] != expected_iterations:
        raise ValueError('Trial did not process its full training budget')
    last = metrics[-1]
    if [last['val/center/' + name + '_GT'] for name in ('vehicle', 'pedestrian', 'obstacle')] != [341., 687., 674.]:
        raise ValueError('Not the complete selected merged 745-frame validation set')
    return dict(root=str(root), contract=manifest['contract_sha256'], identity=manifest['identity'],
                status=status, metrics=metrics)


def compare(baseline, candidate):
    result = dict(baseline=summarize(baseline), candidate=summarize(candidate))
    identities = []
    for name in ('baseline', 'candidate'):
        identity = json.loads(json.dumps(result[name]['identity']))
        identity.pop('initial_checkpoint_sha256')
        identities.append(identity)
    if identities[0] != identities[1]:
        differences = [key for key in set(identities[0]) | set(identities[1])
                       if identities[0].get(key) != identities[1].get(key)]
        raise ValueError('Paired inputs differ beyond initialization: ' + repr(differences))
    if result['baseline']['status']['epoch'] != result['candidate']['status']['epoch']:
        raise ValueError('Compare the same epoch budget')
    result['only_identity_difference'] = 'initial_checkpoint_sha256'
    result['delta_percentage_points'] = {
        key: 100 * (result['candidate']['metrics'][-1][key] - value)
        for key, value in result['baseline']['metrics'][-1].items()
        if key.endswith('mean_AP') or key == 'val/center/mAP'}
    result['scope'] = 'Single seed, same generated validation, early convergence only; not final rosbag accuracy'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.baseline, args.candidate)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['delta_percentage_points'], indent=2))


if __name__ == '__main__':
    main()
