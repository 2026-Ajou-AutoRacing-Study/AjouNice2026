#!/usr/bin/env python3
"""Generate a new MORAI test network; never edit the original or load MORAI."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def without_gt(source):
    result = copy.deepcopy(source)
    for ego in result['egoNetworkData']:
        for entry in ego.get('listEgoNetworkInfo', []):
            kind = int(entry['PUBSUB_TYPE'])
            # Keep only allowed status receivers plus Ctrl Cmd; retain metadata.
            if kind not in (261, 295):
                entry['config']['commType'] = 0
            if kind != 771:
                entry['EgoCtrlConfig']['commType'] = 0
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('local', 'lan', 'teamlead'), default='local')
    parser.add_argument('--output-dir', required=True, type=Path, help='New directory only; refuses overwrite')
    args = parser.parse_args()
    suffix = 'lan_jihun' if args.profile == 'teamlead' else args.profile
    source = ROOT / ('configs/network/ego_network_competition_' + suffix + '.json')
    raw = source.read_bytes()
    data = without_gt(json.loads(raw))
    args.output_dir.mkdir(parents=True, exist_ok=False)
    target = args.output_dir / 'ego_network_no_gt.json'
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    receipt = dict(profile=args.profile, source=str(source), target=str(target.resolve()),
                   source_sha256=hashlib.sha256(raw).hexdigest(),
                   target_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                   allowed_network_types=[261, 295, 771],
                   sensor_file_changed=False, loaded_into_morai=False)
    (args.output_dir / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
