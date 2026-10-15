#!/usr/bin/env python3
"""Read-only full-stack delivery audit; no detector rerun or safety claim."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'perception/detection3d/bevfusion/tools'))
from audit_base5_predicted_path_selection import audit as audit_paths, stats
from audit_recorded_perception_pipeline import audit as audit_delivery
from review_tracker_bag_inventory import inspect

TRACK = '/perception/object_recognition/tracking/objects'
PRED = '/perception/object_recognition/objects'
DET = '/perception/object_recognition/detection/objects'
TRAJ = '/planning/trajectory'


def stamp_key(msg):
    # rospy Publisher assigns a separate seq per topic. It is not inherited identity.
    return (msg.header.stamp.to_nsec(), msg.header.frame_id,
            tuple(bytes(o.object_id.uuid).hex() for o in msg.objects))


def intervals(values):
    """Arrival cadence: retain duplicate/out-of-order header stamps elsewhere."""
    values = sorted(values)
    delta = np.diff(values).tolist()
    return dict(messages=len(values), hz=(len(values)-1)/(values[-1]-values[0])
                if len(values) > 1 and values[-1] > values[0] else None,
                gaps_sec=stats(delta), gaps_gt025=sum(d > .25 for d in delta),
                gaps_gt05=sum(d > .5 for d in delta))


def run_window(events):
    start = next((e['time'] for e in events if e['event'] == 'CONTROLLER_START'), None)
    end = next((e['time'] for e in events if e['event'] == 'RUN_FINISHED'), None)
    return tuple(datetime.fromisoformat(s).timestamp() for s in (start, end)) if start and end else None


def audit(run, output):
    import rosbag
    if output.exists():
        raise FileExistsError('Use a fresh analysis directory')
    bags = list(run.glob('*.bag'))
    if len(bags) != 1:
        raise ValueError('Expected one finalized bag per run directory')
    bag_path = bags[0]
    sidecar = json.loads(bag_path.with_suffix('.json').read_text())
    events = [json.loads(s) for s in (run / 'events.jsonl').read_text().splitlines() if s.strip()]
    window = run_window(events)
    if window is None:
        raise ValueError('No explicit controller run window; do not infer from speed')
    parameters = yaml.safe_load((run / 'ros_parameters.yaml').read_text())
    arrivals = defaultdict(list)
    ages = defaultdict(list)
    zero_stamps = Counter()
    labels = defaultdict(Counter)
    tracks = defaultdict(list)
    predictions = []
    validation = {'active': Counter(), 'outside_run': Counter()}
    validation_latency = defaultdict(list)
    validation_events = []
    logs = []
    map_logs = []
    collision = []
    diagnostics = defaultdict(Counter)
    diagnostic_values = defaultdict(list)
    ego = []
    track_counts = []
    optimizer_events = []
    avoid_reasons = Counter()
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        topic_index = {k: dict(type=v.msg_type, count=v.message_count)
                       for k, v in bag.get_type_and_topic_info().topics.items()}
        topics = [TRACK, PRED, DET, TRAJ, '/Ego_topic', '/CollisionData', '/diagnostics',
                  '/planning/validation_status', '/rosout',
                  '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array']
        for topic, msg, receipt in bag.read_messages(topics=topics):
            t = receipt.to_sec()
            active = window[0] <= t <= window[1]
            if active:
                arrivals[topic].append(t)
            if hasattr(msg, 'header') and active:
                stamp = msg.header.stamp.to_sec()
                if stamp:
                    ages[topic].append(t-stamp)
                else:
                    zero_stamps[topic] += 1
            if topic in (TRACK, PRED, DET):
                if active:
                    for obj in msg.objects:
                        label = max(obj.classification, key=lambda c: c.probability).label if obj.classification else 0
                        labels[topic][int(label)] += 1
                if topic == TRACK:
                    tracks[stamp_key(msg)].append(t)
                    if active:
                        track_counts.append([t-start, len(msg.objects)])
                elif topic == PRED and active:
                    predictions.append((stamp_key(msg), t))
            elif topic == '/Ego_topic' and active:
                ego.append([t-start, msg.position.x, msg.position.y,
                            float(np.hypot(msg.velocity.x, msg.velocity.y))])
            elif topic == '/CollisionData':
                objects = list(msg.collision_object) + list(msg.collision_objecta)
                if objects:
                    collision.append(dict(t=t-start, active=active,
                                          objects=[dict(id=o.unique_id, name=o.name, type=o.type) for o in objects]))
            elif topic == '/planning/validation_status':
                group = 'active' if active else 'outside_run'
                validation[group]['messages'] += 1
                bad = [k for k in msg.__slots__ if k.startswith('is_valid_') and not getattr(msg, k)]
                validation[group].update(bad)
                validation[group]['any_invalid'] += bool(bad)
                validation_latency[group].append(float(msg.latency))
                if bad:
                    validation_events.append(dict(t=t-start, active=active, failed=bad, latency=msg.latency))
            elif topic == '/diagnostics' and active:
                for status in msg.status:
                    diagnostics[status.name][str(status.level)+':'+status.message] += 1
                    for value in status.values:
                        try:
                            diagnostic_values[status.name+'/'+value.key].append(float(value.value))
                        except ValueError:
                            pass
            elif topic == '/rosout':
                row = dict(t=t-start, active=active, node=msg.name, level=msg.level, text=msg.msg)
                if msg.level >= 4:
                    logs.append(row)
                match = re.search(r'objects=(\d+) mapped=(\d+) fallback=(\d+) paths=(\d+) latency=([\d.]+)ms', msg.msg)
                if msg.name == '/map_based_prediction' and match:
                    map_logs.append(dict(t=t-start, active=active, objects=int(match[1]), mapped=int(match[2]),
                                         fallback=int(match[3]), paths=int(match[4]), ms=float(match[5])))
                if 'Invalid MPT QP bounds' in msg.msg or '[MPT] Optimization failed' in msg.msg:
                    optimizer_events.append(row)
            elif topic.endswith('avoidance_debug_message_array') and active:
                for row in msg.avoidance_info:
                    avoid_reasons[('allowed' if row.allow_avoidance else 'not_allowed')+'/'+row.failed_reason] += 1
    # Stamp/frame and ordered object identities preserved; publisher seq is not.
    matched = []
    ambiguous = 0
    for key, t in predictions:
        candidates = tracks.get(key, [])
        if len(candidates) == 1:
            matched.append(t-candidates[0])
        elif len(candidates) > 1:
            ambiguous += 1
    selected_logs = [r for r in map_logs if r['active'] and r['objects']]
    report = dict(
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        inventory=inspect(bag_path), topic_index=topic_index,
        controller_window=dict(start=window[0], end=window[1], duration_sec=window[1]-window[0]),
        validator_result={k:sidecar[k] for k in ('status', 'reason', 'metrics')},
        active_delivery={k:dict(**intervals(v), receive_minus_header_sec=stats(ages[k]),
                                zero_stamps=zero_stamps[k]) for k,v in arrivals.items()},
        active_object_observations={k:dict(v) for k,v in labels.items()},
        exact_header_track_to_prediction=dict(predictions=len(predictions), matched=len(matched),
            ambiguous_keys=ambiguous, receive_delta_sec=stats(matched),
            negative_pairs=sum(v < 0 for v in matched),
            caveat='Two subscriber receipt timestamps, not pure callback or wire latency; exact stamp/frame/ordered UUIDs matched. '
                   'Negative values retained: cross-connection callback ordering may invert recording order.'),
        collision_messages=collision, validation={k:dict(v) for k,v in validation.items()},
        validator_trajectory_header_age_sec={k:stats(v) for k,v in validation_latency.items()},
        validation_events=validation_events, warning_logs=logs, map_callback_logs=map_logs,
        map_nonempty_callback_sample_ms=stats([r['ms'] for r in selected_logs]),
        diagnostics={k:dict(v) for k,v in diagnostics.items()},
        diagnostic_numeric={k:stats(v) for k,v in diagnostic_values.items()},
        avoidance_debug_reasons=dict(avoid_reasons),
        predictor_parameters=parameters.get('map_based_prediction'),
        tracker_parameters=dict(node=parameters.get('ekf_multi_object_tracking'),
                                global_configure=parameters.get('configure'),
                                motion_policy=parameters.get('tracked_object_motion_policy')),
        ego_distance_m=sum(float(np.hypot(b[1]-a[1],b[2]-a[2])) for a,b in zip(ego,ego[1:])),
        ego_speed_mps=stats([v[3] for v in ego]),
        planner_parameters=dict(lane_change_use_all_predicted_paths=[], cruise_outside_count=[], roundabout_min_confidence=[]),
        missing_requested_topics=[t for t in sidecar['recording']['requested_topics'] if t not in topic_index],
        detector_diagnostic_topics=dict(
            canonical='/perception/bevfusion/status',
            base5_trial='/perception/base5_trial/status',
            launch_remap_source='base_ws/src/perception/bevfusion_ros/launch/base5_trial.launch',
            recorded=any(t in topic_index for t in ('/perception/bevfusion/status','/perception/base5_trial/status'))),
        limitations=['No raw camera/LiDAR or measurement-track topic in this recording.',
                     'No low-score candidate recovery or fresh detector measurement age from tracked header.',
                     'ROS arrival age requires a common header/receiver clock; no LAN clock-offset certification.',
                     'Per-module consumed object/path indices absent: selection policy is not a decision trace.',
                     'Map latency logs are 5-second throttled callback/publish samples, not all-frame percentiles.',
                     'Single successful run, no placed shadow-section objects; not universal safety validation.'])
    def walk(value, prefix=''):
        if isinstance(value, dict):
            for k,v in value.items():
                path = prefix+'/'+str(k)
                if k == 'use_all_predicted_paths' and 'lane_change' in path:
                    report['planner_parameters']['lane_change_use_all_predicted_paths'].append(dict(path=path,value=v))
                if k == 'num_of_predicted_paths' and 'outside_obstacle' in path:
                    report['planner_parameters']['cruise_outside_count'].append(dict(path=path,value=v))
                if k == 'min_predicted_path_confidence' and 'roundabout' in path:
                    report['planner_parameters']['roundabout_min_confidence'].append(dict(path=path,value=v))
                walk(v,path)
    walk(parameters)
    report['prediction_paths'] = audit_paths(bag_path, report['inventory'].get('recorded_model'))
    report['delivery_all_recording'] = audit_delivery(bag_path)
    output.mkdir(parents=True)
    (output/'audit.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3,1,figsize=(12,8),sharex=True)
    axes[0].plot([v[0] for v in ego],[v[3]*3.6 for v in ego],lw=1)
    axes[0].set_ylabel('Ego km/h')
    axes[1].plot([v[0] for v in track_counts],[v[1] for v in track_counts],lw=1)
    axes[1].set_ylabel('Tracked objects')
    axes[2].scatter([v['t'] for v in map_logs],[v['ms'] for v in map_logs],s=20,label='Throttled Map-CV callback')
    axes[2].set_ylabel('Sampled callback ms')
    axes[2].set_xlabel('Seconds since recording start')
    for axis in axes:
        for event in optimizer_events:
            if event['active']:
                axis.axvline(event['t'],color='red',alpha=.25,lw=1)
        axis.grid(alpha=.2)
    axes[2].legend()
    fig.suptitle('Recorded full-stack run: red = MPT solver warnings (not collision)')
    fig.tight_layout()
    fig.savefig(output/'overview.png',dpi=160)
    plt.close(fig)
    print(json.dumps({k:report[k] for k in ('controller_window','active_object_observations',
                      'exact_header_track_to_prediction','map_nonempty_callback_sample_ms','validation')},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    audit(args.run,args.output)
