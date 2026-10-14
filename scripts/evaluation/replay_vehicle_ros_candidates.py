#!/usr/bin/env python3
"""Private ROS qualification of frozen vehicle candidates with honest stamps.

Uses every valid prehistory frame, original input header and receive schedule,
one shared input topic, and a common 100Hz simulation clock. Real native nodes
own their 30Hz output schedulers; no simulator/GPU/planner is launched.
"""
import argparse
from bisect import bisect_left
import gzip
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
import threading
import hashlib

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/src'),
               str(ROOT / 'base_ws/src/perception_object_pipeline/src')]


def validate_private_port(port):
    if not isinstance(port, int) or isinstance(port, bool) or not 20000 <= port <= 65535:
        raise ValueError('Research master must use an unused private port in 20000..65535')
    with socket.socket() as check:
        check.bind(('127.0.0.1', port))


def validate_output_contract(report):
    count = report['sent_frames']
    if count != report['valid_source_frames'] or any(n < count for n in report['outputs'].values()):
        raise RuntimeError('Incomplete actual ROS input/output sequence')
    if any(n != count for n in report['native_arrival_counts'].values()):
        raise RuntimeError('A native subscriber did not acknowledge every original input')
    for row in report['paired'].values():
        if row['unpaired_frames'] or row['identity_set_changed_frames']:
            raise RuntimeError('Guard-only scheduler/identity cohort changed; qualification failed')
        maximum = row['stats']['center_delta']['maximum']
        if maximum is not None and maximum > 1.e-9:
            raise RuntimeError('Guard-only comparison changed native center state; qualification failed')
        maximum = row['stats']['stamp_delta']['maximum']
        if maximum is not None and maximum > 1.e-9:
            raise RuntimeError('Guard-only comparison changed native output stamps; qualification failed')


def compare_wire_bags(reference_path, candidate_path, topics):
    """Exact serialized TrackedObjects comparison, including covariance/flags."""
    import rosbag
    def read(path):
        values = {topic: [] for topic in topics}
        with rosbag.Bag(str(path)) as bag:
            for topic, raw, _ in bag.read_messages(topics=topics, raw=True):
                values[topic].append(raw[1])
        return values
    reference, candidate = read(reference_path), read(candidate_path)
    results = {}
    for topic in topics:
        before, after = reference[topic], candidate[topic]
        changed = [i for i, (a, b) in enumerate(zip(before, after)) if a != b]
        results[topic] = dict(reference_messages=len(before), candidate_messages=len(after),
            different_serialized_messages=len(changed), first_changed_index=changed[0] if changed else None,
            reference_wire_sha256=hashlib.sha256(b''.join(before)).hexdigest(),
            candidate_wire_sha256=hashlib.sha256(b''.join(after)).hexdigest(),
            exact=bool(before) and before == after)
    return results


def direction_summary(frames, identity, start, end):
    selected = []
    for frame in frames:
        if start <= frame['stamp'] <= end:
            for obj in frame['objects']:
                if obj['id'] == identity:
                    selected.append(dict(stamp=frame['stamp'], center=obj['center'],
                        yaw=obj['yaw'], velocity=obj['world_velocity']))
    wrong = [r for r in selected if r['velocity'][1] > 0.5]
    return dict(object_frames=len(selected), opposite_world_y_frames=len(wrong),
        opposite_start=wrong[0]['stamp'] if wrong else None,
        opposite_end=wrong[-1]['stamp'] if wrong else None, rows=selected)


def paired_summary(before, after, max_dt=.011):
    from audit_tracking_continuity import stats
    times = [f['stamp'] for f in after]
    rows = []; missing = 0; changed = 0
    for reference in before:
        index = bisect_left(times, reference['stamp'])
        choices = [i for i in (index-1, index) if 0 <= i < len(times)]
        if not choices:
            missing += 1; continue
        chosen = min(choices, key=lambda i: abs(times[i] - reference['stamp']))
        candidate = after[chosen]
        if abs(candidate['stamp'] - reference['stamp']) > max_dt:
            missing += 1; continue
        left = {o['id']: o for o in reference['objects']}
        right = {o['id']: o for o in candidate['objects']}
        if set(left) != set(right): changed += 1
        for identity in set(left) & set(right):
            a, b = left[identity], right[identity]
            rows.append(dict(stamp_delta=abs(candidate['stamp']-reference['stamp']),
                center_delta=math.dist(a['center'][:2], b['center'][:2]),
                world_velocity_delta=math.dist(a['world_velocity'], b['world_velocity'])))
    return dict(reference_frames=len(before), unpaired_frames=missing,
        identity_set_changed_frames=changed, paired_object_rows=len(rows),
        stats={k: stats([r[k] for r in rows]) for k in
               ('stamp_delta', 'center_delta', 'world_velocity_delta')})


def recorded_sonata_handoff(frames):
    """Known UUID diagnostics only; not a same-actor GT identity certificate."""
    from audit_tracking_continuity import stats
    old = '414a4f5500000000000000000000002c'
    new = '414a4f55000000000000000000000036'
    base = 1791908591.4141977
    overlap = []; old_times = []; new_times = []; separations = []
    for frame in frames:
        if not base+133.5 <= frame['stamp'] <= base+136.5: continue
        actors = {o['id']: o for o in frame['objects']}
        if old in actors: old_times.append(frame['stamp'])
        if new in actors: new_times.append(frame['stamp'])
        if old in actors and new in actors:
            overlap.append(frame['stamp'])
            separations.append(math.dist(actors[old]['center'][:2], actors[new]['center'][:2]))
    return dict(old_id=old, new_id=new, old_object_frames=len(old_times),
        new_object_frames=len(new_times), overlap_frames=len(overlap),
        overlap_span_sec=overlap[-1]-overlap[0] if overlap else 0.,
        new_first_stamp=new_times[0] if new_times else None,
        old_last_stamp=old_times[-1] if old_times else None,
        center_separation_m=stats(separations),
        scope='Recorded-event UUID diagnostics; verify actor correspondence separately before claiming recovery.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=29377)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--measurement-clock', action='store_true',
                        help='Separate research policy: decision at measurement time, copy age capped at 0.5s')
    parser.add_argument('--reference-bag', type=Path,
                        help='Require all four serialized output streams to match a frozen earlier nominal replay')
    args = parser.parse_args()
    if args.output.exists() or not 0.5 <= args.rate <= 1.2:
        raise ValueError('Fresh output and bounded real-time replay rate required')
    validate_private_port(args.port)
    source_frames = json.load(gzip.open(args.frames, 'rt'))
    frames = [f for f in source_frames if f['world_objects'] is not None]
    if len(frames) < 100:
        raise ValueError('Full prehistory sequence required, not a cropped event')
    if any(b['stamp'] <= a['stamp'] or b['receipt'] < a['receipt']
           for a, b in zip(frames, frames[1:])):
        raise ValueError('Input header/receipt order must remain original')
    build = json.loads((args.build / 'build_receipt.json').read_text())
    args.output.mkdir(parents=True)
    previous_ros_environment = {key: os.environ.get(key) for key in ('ROS_MASTER_URI', 'ROS_IP')}
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(args.port)
    os.environ['ROS_IP'] = '127.0.0.1'
    from planning_scenario_validator.runtime import Processes
    from audit_tracking_continuity_events import sha
    processes = Processes(args.output / 'logs', os.environ.copy())
    modes = ('baseline', 'guard', 'innovation', 'combined')
    output = {m: [] for m in modes}; guarded = {m: [] for m in modes}
    arrivals = {m: 0.0 for m in modes}; cycles = {m: 0.0 for m in modes}
    arrival_counts = {m: 0 for m in modes}
    acknowledgements = threading.Condition()
    subscriptions = []
    try:
        processes.start('master', ['roscore', '-p', str(args.port)])
        import rosgraph
        deadline = time.monotonic() + 15
        while not rosgraph.is_master_online():
            if time.monotonic() > deadline: raise RuntimeError('Private ROS master unavailable')
            time.sleep(.1)
        import rospy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Header
        from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
        from perception_object_pipeline.detected_objects import make_detected_objects
        from audit_tracking_continuity import object_row, output_identity_summary
        rospy.init_node('vehicle_candidate_ros_replay', disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        clock = rospy.Publisher('/clock', Clock, queue_size=100, latch=True)
        first = min(frames[0]['stamp'], frames[0]['receipt'])
        last = frames[-1]['receipt'] + .55
        clock.publish(Clock(rospy.Time.from_sec(first)))
        publisher = rospy.Publisher('/vehicle_ros/detection', DetectedObjects, queue_size=50)
        def receive(message, key, diagnostics=False):
            objects = []
            for native in message.objects:
                row = object_row(native)
                vx = native.kinematics.twist_with_covariance.twist.linear.x
                vy = native.kinematics.twist_with_covariance.twist.linear.y
                c, s = math.cos(row['yaw']), math.sin(row['yaw'])
                row['world_velocity'] = [c*vx-s*vy, s*vx+c*vy]
                row['body_velocity'] = [vx, vy]
                row['yaw_rate'] = native.kinematics.twist_with_covariance.twist.angular.z
                objects.append(row)
            destination = guarded if diagnostics else output
            destination[key].append(dict(stamp=message.header.stamp.to_sec(), objects=objects))
        configuration = yaml.safe_load((ROOT / 'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())
        configuration['configure'].update(fixed_output_rate_hz=30.,
            maximum_fixed_output_prediction_age_sec=.5, pedestrian_gaussian_association_ranking=True,
            center_motion_velocity_fusion=True, center_motion_fuse_ekf_state=False,
            center_motion_override_output_velocity=False, compensate_output_to_current_time=False)
        rospy.set_param('/topic_name', dict(lidar_objects='/vehicle_ros/detection', odometry='/unused_odometry',
            output_track_jsk='/output_track_jsk', output_track_marker='/output_track_marker'))
        def acknowledge(message, key, values):
            with acknowledgements:
                values[key] = message.stamp.to_sec()
                if values is arrivals: arrival_counts[key] += 1
                acknowledgements.notify_all()
        def wait_barrier(values, target):
            deadline = time.monotonic() + 3.0
            with acknowledgements:
                while any(value + 1.e-7 < target for value in values.values()):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        raise RuntimeError('Private native boundary ACK missing: ' + str((values, target)))
                    acknowledgements.wait(min(.1, remaining))
        for mode in modes:
            namespace = '/vehicle_ros/' + mode
            for key, value in configuration.items(): rospy.set_param(namespace + '/' + key, value)
            subscriptions.append(rospy.Subscriber(namespace + '/tracking', TrackedObjects,
                lambda msg, key=mode: receive(msg, key), queue_size=1000))
            subscriptions.append(rospy.Subscriber(namespace + '/guarded', TrackedObjects,
                lambda msg, key=mode: receive(msg, key, True), queue_size=1000))
            subscriptions.append(rospy.Subscriber(namespace + '/arrived', Header,
                lambda msg, key=mode: acknowledge(msg, key, arrivals), queue_size=100))
            subscriptions.append(rospy.Subscriber(namespace + '/cycle', Header,
                lambda msg, key=mode: acknowledge(msg, key, cycles), queue_size=100))
            binary = build['binaries']['innovation' if mode in ('innovation', 'combined') else 'baseline']
            if sha(Path(binary)) != build['binaries_sha256']['innovation' if mode in ('innovation', 'combined') else 'baseline']:
                raise RuntimeError('Built candidate binary changed')
            processes.start(mode, [binary, '__name:=tracker', '__ns:=' + namespace,
                '_world_frame_id:=map', '_research_direction_guard:=' + str(mode in ('guard', 'combined')).lower(),
                '_research_guard_measurement_clock:=' + str(args.measurement_clock).lower(),
                '~input/objects:=/vehicle_ros/detection', '~output/objects:=' + namespace + '/tracking',
                '~diagnostics/direction_guarded_objects:=' + namespace + '/guarded',
                '~diagnostics/input_arrived:=' + namespace + '/arrived',
                '~diagnostics/cycle_complete:=' + namespace + '/cycle',
                '/output_track_jsk:=' + namespace + '/jsk', '/output_track_marker:=' + namespace + '/marker'])
        deadline = time.monotonic() + 20
        while publisher.get_num_connections() != len(modes):
            if time.monotonic() > deadline: raise RuntimeError('All isolated nodes must subscribe to shared input')
            time.sleep(.05)
        topics = ['/clock', '/vehicle_ros/detection', '/rosout']
        topics += ['/vehicle_ros/' + m + '/' + t for m in modes for t in ('tracking', 'guarded')]
        processes.start('recorder', ['rosbag', 'record', '--lz4', '-O', str(args.output / 'replay.bag'), *topics])
        time.sleep(.5)
        index = tick = 0; wall_start = time.monotonic(); tick_sec = .01
        first_ros = rospy.Time.from_sec(first)
        first_nanoseconds = first_ros.to_nsec()
        while first + tick*tick_sec <= last:
            # Exact 10,000,000ns increments avoid epoch-scale float rounding
            # leaving one native ros::Rate asleep until the following tick.
            clock_nanoseconds = first_nanoseconds + tick*10000000
            clock_time = rospy.Time(clock_nanoseconds//1000000000, clock_nanoseconds%1000000000)
            current = clock_time.to_sec()
            # Reproduce original callback availability, not header-time input:
            # the input age and 30Hz predicted-copy behavior are kept explicit.
            while index < len(frames) and frames[index]['receipt'] <= current:
                frame = frames[index]
                boxes = [o['center'][:2] + [o['center'][2] + o['size'][2]*.5] + o['size'] + [o['yaw']]
                         for o in frame['world_objects']]
                message = make_detected_objects(Header(seq=index,
                    stamp=rospy.Time.from_sec(frame['stamp']), frame_id='map'),
                    boxes, [o['score'] for o in frame['world_objects']],
                    [o['class_name'] for o in frame['world_objects']])
                publisher.publish(message)
                wait_barrier(arrivals, frame['stamp'])
                index += 1
            clock.publish(Clock(clock_time))
            # Initial native loops may have completed before the diagnostic
            # subscriber connected, then sleep until the first advanced tick.
            if tick: wait_barrier(cycles, current)
            tick += 1
            time.sleep(max(0., wall_start + tick*tick_sec/args.rate - time.monotonic()))
        time.sleep(.2)
        processes.stop('recorder')
        report = dict(source_frames=str(args.frames), source_sha256=sha(args.frames),
            sent_frames=index, valid_source_frames=len(frames), source_frames_total=len(source_frames),
            dropped_source_tf_frames=len(source_frames)-len(frames), input_age_preserved=True,
            original_header_preserved=True, original_receipt_schedule_preserved=True,
            private_boundary_barriers=True, receipt_schedule_quantization_sec=.01,
            direction_guard_clock='measurement decision + 0.5s copy age' if args.measurement_clock else 'publication-time 0.4s hold',
            native_arrival_counts=arrival_counts,
            simulation_duration_sec=last-first, wall_duration_sec=time.monotonic()-wall_start,
            outputs={m: len(v) for m, v in output.items()},
            identities={m: output_identity_summary(v) for m, v in output.items()},
            guarded_object_frames={m: sum(len(f['objects']) for f in guarded[m]) for m in modes},
            paired={'baseline_guard': paired_summary(output['baseline'], output['guard']),
                    'innovation_combined': paired_summary(output['innovation'], output['combined'])},
            kona={m: direction_summary(v, '414a4f55000000000000000000000030',
                1791908591.4141977+123.8, 1791908591.4141977+125.2) for m, v in output.items()},
            sonata_handoff={m: recorded_sonata_handoff(v) for m, v in output.items()},
            scope='Actual isolated ROS nodes with instrumented arrival/cycle boundary barriers, full frozen world input, original header/receipt age quantized within 10ms, native 30Hz scheduler. Deterministic logic qualification, not wall-time scheduling/TF/GPU/MORAI/planner or closed-loop certification.')
        if args.reference_bag:
            report['nominal_wire_reference'] = dict(path=str(args.reference_bag), sha256=sha(args.reference_bag),
                topics=compare_wire_bags(args.reference_bag, args.output/'replay.bag',
                    ['/vehicle_ros/'+m+'/tracking' for m in modes]))
        with gzip.open(args.output / 'outputs.json.gz', 'wt') as handle: json.dump(output, handle)
        with gzip.open(args.output / 'guarded.json.gz', 'wt') as handle: json.dump(guarded, handle)
        (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        validate_output_contract(report)
        if args.reference_bag and not all(v['exact'] for v in report['nominal_wire_reference']['topics'].values()):
            raise RuntimeError('Nominal serialized ROS output differs from the pre-fix frozen reference')
        print(json.dumps({k: report[k] for k in ('sent_frames', 'outputs', 'guarded_object_frames', 'paired')}, indent=2))
    finally:
        for subscription in subscriptions: subscription.unregister()
        processes.close()
        for key, value in previous_ros_environment.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value


if __name__ == '__main__': main()
