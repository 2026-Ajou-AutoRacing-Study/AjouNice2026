#!/usr/bin/env python3
"""Read indexed bag headers and existing per-case contracts, without replay/GPU.

Existing comparison cases are development data, not newly held-out test bags.
No full scan, model inference or changes to historical evaluation defaults.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def inventory(roots):
    import rosbag
    cases, seen = [], set()
    for root in roots:
        for path in sorted(Path(root).glob('*/case_contract.json')):
            contract = json.loads(path.read_text())
            bag_path = Path(contract['raw_bag'])
            identity = str(bag_path.resolve())
            if identity in seen:
                raise ValueError('Duplicate raw bag in case inventory: ' + identity)
            seen.add(identity)
            row = dict(case=path.parent.name, mission=contract['mission'], raw_bag=identity,
                       exists=bag_path.is_file(), contract=str(path),
                       contract_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                       heading_correction_deg=contract.get('heading_correction_deg'),
                       role='dev_comparison_previously_evaluated',
                       asset_name_map=contract.get('asset_name_map'))
            row['asset_name_map_exists'] = bool(row['asset_name_map'] and Path(row['asset_name_map']).is_file())
            if row['exists']:
                with rosbag.Bag(str(bag_path), 'r') as bag:
                    duration = bag.get_end_time() - bag.get_start_time()
                    topics = bag.get_type_and_topic_info().topics
                    row.update(bytes=bag_path.stat().st_size, duration_sec=duration,
                               topics={name: dict(type=t.msg_type, count=t.message_count,
                                                  header_average_hz=t.frequency)
                                       for name, t in topics.items() if name.startswith('/sensor/') or
                                       name in ('/Ego_topic', '/Object_topic', '/dataset/scenario_marker')})
                names = set(row['topics'])
                required = {'/sensor/lidar1', '/sensor/camera/front/compressed', '/sensor/camera/front_left/compressed',
                            '/sensor/camera/front_right/compressed', '/sensor/camera/back/compressed', '/Ego_topic', '/Object_topic'}
                row['missing_required_topics'] = sorted(required - names)
                row['has_collection_markers'] = '/dataset/scenario_marker' in names
            cases.append(row)
    return dict(cases=cases, mission_counts=dict(Counter(r['mission'] for r in cases)),
                count=len(cases), missing_bags=sum(not r['exists'] for r in cases),
                total_bytes=sum(r.get('bytes', 0) for r in cases),
                limitations=['Indexed headers only: no synchronization or GT geometry certification.',
                             'These bags already influenced prior comparisons, so are dev-val, not untouched test.',
                             'Header topic frequency is not proof of continuous detections or frame gaps.',
                             'Marker-based Base3 extraction cannot blindly consume marker-free mission bags.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract-root', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Fresh output required')
    report = inventory(args.contract_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'cases'}, ensure_ascii=False))
