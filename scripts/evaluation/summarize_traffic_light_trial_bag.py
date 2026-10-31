#!/usr/bin/env python3
"""Summarize recorded pipeline observations, not accuracy or signal compliance."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    values = sorted(values)
    if not values:
        return dict(n=0)
    return dict(n=len(values), mean=statistics.mean(values), p50=statistics.median(values),
                p90=values[math.ceil(.90 * len(values))-1],
                p95=values[math.ceil(.95 * len(values))-1], max=values[-1])


def summarize(path):
    import rosbag
    grouped = defaultdict(Counter)
    timings, matched_timings, times = defaultdict(list), defaultdict(list), []
    transitions, previous = [], object()
    projection_reasons, unknown = defaultdict(Counter), Counter()
    allowed_heads = {201351: {'C1256W000077'}, 201365: {'C1256W000091'},
                     201290: {'C1256W000016', 'C1256W000018'}, 201326: {'C1256W000052'}}
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=[
                '/traffic_light_node/debug_selection', '/traffic_light_node/debug_classifier',
                '/traffic_light_node/debug_runtime', '/perception/traffic_light_recognition/map_rois',
                '/perception/traffic_light_recognition/route_group_id']):
            elapsed = stamp.to_sec() - start
            if topic.endswith('/route_group_id'):
                if msg.data != previous:
                    transitions.append(dict(t=elapsed, group=msg.data)); previous = msg.data
                continue
            row = json.loads(msg.data)
            group = row.get('group_id')
            if topic.endswith('/debug_selection'):
                grouped[group]['frames'] += 1
                grouped[group]['matched' if row.get('status') == 'matched' else 'missing'] += 1
                if row.get('status') == 'matched':
                    grouped[group]['head:' + str(row.get('head_id'))] += 1
            elif topic.endswith('/debug_classifier'):
                unknown[(group, row.get('reason'))] += 1
            elif topic.endswith('/map_rois'):
                for head in row.get('rejected_heads', []):
                    if head.get('head_id') in allowed_heads.get(group, set()):
                        projection_reasons[group][head.get('reason')] += 1
            else:
                times.append(stamp.to_sec())
                for key, value in (row.get('frame_timing') or {}).items():
                    if key.endswith('_sec') and isinstance(value, (int, float)):
                        timings[key[:-4] + '_ms'].append(value * 1000)
                        if (row.get('frame_decision') or {}).get('selection_status') == 'matched':
                            matched_timings[key[:-4] + '_ms'].append(value * 1000)
    return dict(schema='traffic_light_trial_observations_v1', bag=str(path),
                observations={str(k): dict(v) for k, v in grouped.items()},
                route_transitions=transitions,
                mission_head_projection_rejections={str(k): dict(v) for k, v in projection_reasons.items()},
                classifier_reasons={str(k): v for k, v in unknown.items()},
                timing_ms={key: distribution(values) for key, values in timings.items()},
                matched_frame_timing_ms={key: distribution(values) for key, values in matched_timings.items()},
                processed_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
                limitations=['No native-state or independent image-identity GT in this summary.',
                             'All route-active frames include distant/out-of-view approaches and long stops.',
                             'Match counts are not classification accuracy, visible-frame recall, or driving PASS.',
                             'Callback wall time excludes debug-image encoding and is not full-stack end-to-end age.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    args.output.resolve().relative_to(root / 'artifacts/evaluation')
    if args.output.exists():
        parser.error('Use a fresh output; prior evidence is preserved')
    result = summarize(args.bag.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2); stream.write('\n')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
