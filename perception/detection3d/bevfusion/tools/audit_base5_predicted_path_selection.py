#!/usr/bin/env python3
"""Read recorded futures; emulate selection only, never planner collision decisions."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

TOPIC = '/perception/object_recognition/objects'
VEHICLES = {1, 2, 3, 4}


def source_contract(root):
    import yaml
    launch = root/'planning_ws/src/autoware_launch/config/ros1/planning'
    lc = yaml.safe_load((launch/'behavior_path_modules.yaml').read_text())['lane_change']['collision_check']['use_all_predicted_paths']
    rnd = yaml.safe_load((launch/'behavior_velocity_modules.yaml').read_text())['roundabout']['collision_detection']['min_predicted_path_confidence']
    cruise = yaml.safe_load((launch/'motion_velocity_modules.yaml').read_text())['obstacle_cruise/obstacle_filtering/outside_obstacle/num_of_predicted_paths']
    # Fail closed if this diagnostic's fixed policy no longer describes source.
    if lc is not False or cruise != 1 or rnd != .05:
        raise ValueError('Selection source contract changed; update emulation before reporting')
    names = [
        'planning_ws/src/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp',
        'planning_ws/src/autoware_behavior_path_planner_common/src/utils/path_safety_checker/safety_core.cpp',
        'planning_ws/src/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp',
        'planning_ws/src/autoware_behavior_velocity_roundabout_module/src/experimental/scene_roundabout_collision.cpp',
        'planning_ws/src/autoware_launch/config/ros1/planning/behavior_path_modules.yaml',
        'planning_ws/src/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml',
        'planning_ws/src/autoware_launch/config/ros1/planning/motion_velocity_modules.yaml',
    ]
    return dict(lc_use_all_paths=lc, cruise_outside_path_count=cruise,
                roundabout_min_confidence=rnd,
                sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names},
                runtime_parameters_verified=False)


def stats(values):
    if not values:
        return dict(n=0, mean=None, p50=None, p90=None, p95=None, maximum=None)
    return dict(n=len(values), mean=float(np.mean(values)), p50=float(np.median(values)),
                p90=float(np.percentile(values, 90)), p95=float(np.percentile(values, 95)),
                maximum=float(max(values)))


def selected_indices(confidences, mode, threshold=.05):
    """LC keeps exact maximum ties. Cruise tie order is not guaranteed in C++."""
    if not confidences:
        return []
    if not all(math.isfinite(c) for c in confidences):
        raise ValueError('Nonfinite path confidence')
    if mode == 'lc_max_ties':
        highest = max(confidences)
        return [i for i, c in enumerate(confidences) if c == highest]
    if mode == 'cruise_one':
        return [max(range(len(confidences)), key=confidences.__getitem__)]
    if mode == 'roundabout_threshold':
        return [i for i, c in enumerate(confidences) if c >= threshold]
    raise ValueError(mode)


def sample(path, seconds):
    points, dt = path['points'], path['dt']
    if not points or dt <= 0 or seconds < 0:
        return None
    index = seconds / dt
    if index > len(points) - 1 + 1.e-6:
        return None
    index = min(index, len(points) - 1)
    lo = int(index); hi = min(lo + 1, len(points) - 1); alpha = index - lo
    return tuple((1 - alpha) * a + alpha * b for a, b in zip(points[lo], points[hi]))


def separation(first, second, first_seconds, second_seconds):
    a, b = sample(first, first_seconds), sample(second, second_seconds)
    return None if a is None or b is None else math.hypot(a[0] - b[0], a[1] - b[1])


def duration_seconds(value):
    return float(value.sec) + float(value.nanosec) * 1.e-9


def audit(path, model=None):
    import rosbag
    counts = Counter(); path_counts = Counter(); lc_counts = Counter(); round_counts = Counter()
    separation3 = []; revision1 = []; revision3 = []; examples = []; histories = {}
    debug_topics = {}
    with rosbag.Bag(str(path), 'r') as bag:
        for topic, info in bag.get_type_and_topic_info().topics.items():
            if ('roundabout' in topic or 'lane_change' in topic or 'cruise' in topic):
                debug_topics[topic] = dict(type=info.msg_type, count=info.message_count)
        for _, msg, _ in bag.read_messages(topics=[TOPIC]):
            counts['messages'] += 1
            stamp = msg.header.stamp.to_sec()
            for obj in msg.objects:
                label = max(obj.classification, key=lambda c: c.probability).label if obj.classification else 0
                if label not in VEHICLES:
                    continue
                counts['vehicle_observations'] += 1
                paths = [dict(confidence=float(p.confidence), dt=duration_seconds(p.time_step),
                              points=[(v.position.x, v.position.y) for v in p.path])
                         for p in obj.kinematics.predicted_paths]
                if any(not math.isfinite(p['confidence']) for p in paths):
                    counts['invalid_confidence'] += 1
                    continue
                conf = [p['confidence'] for p in paths]
                path_counts[len(paths)] += 1
                lc = selected_indices(conf, 'lc_max_ties')
                rnd = selected_indices(conf, 'roundabout_threshold')
                lc_counts[len(lc)] += 1; round_counts[len(rnd)] += 1
                if not paths:
                    continue
                ordered = sorted(range(len(paths)), key=lambda i: conf[i], reverse=True)
                top = ordered[0]; ident = bytes(obj.object_id.uuid).hex()
                if len(paths) > 1:
                    counts['multiple_paths'] += 1
                    if len(lc) > 1:
                        counts['exact_max_ties'] += 1
                    if conf[top] - conf[ordered[1]] <= .02:
                        counts['near_ties_002'] += 1
                    d = separation(paths[top], paths[ordered[1]], 3., 3.)
                    if d is not None:
                        separation3.append(d)
                        if d > 2.:
                            counts['top2_separation3_gt2m'] += 1
                            if len(examples) < 8:
                                examples.append(dict(id=ident, stamp=stamp, confidences=conf,
                                                     separation_at3_m=d))
                # Same ID/frame, <=0.25s gaps only. Compare at the SAME future time.
                # This is forecast revision, NOT a ground-truth error or a lane-ID switch.
                old = histories.get(ident)
                if old and old['frame'] == msg.header.frame_id and 0 < stamp - old['stamp'] <= .25:
                    delta = stamp - old['stamp']
                    for horizon, values in ((1., revision1), (3., revision3)):
                        d = separation(old['path'], paths[top], horizon, horizon - delta)
                        if d is not None:
                            values.append(d)
                histories[ident] = dict(stamp=stamp, frame=msg.header.frame_id, path=paths[top])
    return dict(path=str(path.resolve()), recorded_model=model,
                counts=dict(counts), published_path_count=dict(path_counts),
                lc_max_tie_count=dict(lc_counts), roundabout_conf005_count=dict(round_counts),
                top2_separation_at3_m=stats(separation3),
                top1_common_future_revision1_m=stats(revision1),
                top1_common_future_revision3_m=stats(revision3), examples=examples,
                planner_debug_topics=debug_topics,
                caveat='Recorded outputs, not current E24 rerun. Selection is source-policy emulation, '
                       'not actual planner use, safety filtering, collision or GT intent evaluation. '
                       'Cruise exact-tie choice is unspecified; revisions use a stable diagnostic representative.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags', nargs='+', type=Path, required=True)
    parser.add_argument('--inventory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Use a fresh report path')
    inventory = {r['path']: r for r in json.loads(args.inventory.read_text())['rows']}
    rows = []
    for path in args.bags:
        result = audit(path, inventory.get(str(path.resolve()), {}).get('recorded_model'))
        rows.append(result)
        print(path.name, result['counts'], flush=True)
    source = Path(__file__)
    report = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), rows=rows,
                  planner_source_contract=source_contract(source.resolve().parents[4]),
                  production_changed=False,
                  limitation='No source/parameter/runtime provenance inferred from bag filename. '
                             'Recorded model receipts are not independent server weight proof.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
