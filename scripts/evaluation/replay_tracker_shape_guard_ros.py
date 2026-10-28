#!/usr/bin/env python3
"""Isolated, ACK-stepped actual ROS replay of the opt-in shape ranking.

No live master, installed files, simulator or production parameters are used.
Each policy owns a separate localhost master. Cached measurement times are retained;
the explicitly synthetic current clock advances on an identical 30Hz grid.
The C++ 100Hz main loop wakes at each grid step; input receipt and every stage's
output are acknowledged before the next step. This is not wall-rate/LAN testing.
"""
from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import io
import json
import math
import os
from pathlib import Path
import pickle
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

import yaml

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'base_ws/src/perception_object_pipeline'
TRACKER = ROOT / 'base_ws/src/ekf-multi-object-tracking'
DEFAULT_CACHE = ROOT / 'planner_logs/analysis/261024_035142_full_route/duplicate_rootcause/world_cache.pkl'
TOPICS = dict(measurement='/shape_guard/measurement_objects',
              policy='/shape_guard/objects', predicted='/shape_guard/predicted_objects',
              association='/shape_guard/association', detection='/shape_guard/detection')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with Path(path).open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def stats(values):
    if not values:
        return dict(count=0)
    import numpy as np
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite metrics')
    return dict(count=len(a), mean=float(a.mean()), minimum=float(a.min()),
                p50=float(np.percentile(a, 50)), p95=float(np.percentile(a, 95)),
                p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def valid_three_second_path(path, allow_shorter=False):
    count = path['count']
    return ((1 <= count <= 31 if allow_shorter else count == 31) and
            abs(path['time_step'] - .1) <= 1.e-7)


def load_frames(path, limit=None):
    start, topics, excluded = pickle.loads(Path(path).read_bytes())
    source = topics['/perception/object_recognition/detection/objects']
    frames = [dict(stamp=f['stamp'], receipt=f['receipt'], world_objects=f['world_objects'])
              for f in source if f['world_objects'] is not None]
    if limit:
        frames = frames[:limit]
    if not frames or any(b['stamp'] <= a['stamp'] for a, b in zip(frames, frames[1:])):
        raise ValueError('Strictly increasing complete world-frame inputs required')
    return start, frames, excluded


def grid(frames, tail=.3, clock_mode='measurement'):
    """Every input is queued before its own tick, with no packet overwrites."""
    arrival = [f['receipt'] if clock_mode == 'receipt' else f['stamp']+.02 for f in frames]
    first = arrival[0]
    ticks = []
    index = 0
    for tick in range(math.ceil((arrival[-1] + tail - first) * 30) + 1):
        now = first + tick / 30.
        selected = []
        while index < len(frames) and arrival[index] <= now + 1.e-6:
            selected.append(index)
            index += 1
        if len(selected) > 1:
            raise ValueError('More than one input per scheduler tick; refusing latest-packet loss')
        ticks.append((now, selected))
    if index != len(frames):
        raise ValueError('Incomplete scheduler grid')
    return ticks


def wait(predicate, timeout, label, processes=()):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('Owned process exited while waiting for ' + label)
        if predicate():
            return
        time.sleep(.001)
    raise TimeoutError(label)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def wire(message):
    old = message.header.seq
    message.header.seq = 0
    try:
        stream = io.BytesIO()
        message.serialize(stream)
        return stream.getvalue()
    finally:
        message.header.seq = old


def object_row(obj, predicted=False):
    kin = obj.kinematics
    pose = kin.initial_pose_with_covariance.pose if predicted else kin.pose_with_covariance.pose
    twist = kin.initial_twist_with_covariance.twist if predicted else kin.twist_with_covariance.twist
    row = dict(uuid=bytes(obj.object_id.uuid).hex(),
               label=max(obj.classification, key=lambda c: c.probability).label,
               center=[pose.position.x, pose.position.y, pose.position.z],
               velocity_body=[twist.linear.x, twist.linear.y],
               size=[obj.shape.dimensions.x, obj.shape.dimensions.y, obj.shape.dimensions.z])
    if predicted:
        row['paths'] = [dict(confidence=p.confidence, count=len(p.path),
                            time_step=p.time_step.sec + p.time_step.nanosec * 1.e-9)
                        for p in kin.predicted_paths]
    if not all(math.isfinite(v) for key in ('center', 'velocity_body', 'size') for v in row[key]):
        raise ValueError('Nonfinite ROS object')
    return row


def private_build(output):
    snapshot = output / 'tracker_snapshot'
    snapshot.mkdir()
    for name in ('src', 'include', 'config'):
        shutil.copytree(TRACKER / name, snapshot / name)
    for name in ('CMakeLists.txt', 'package.xml'):
        shutil.copy2(TRACKER / name, snapshot / name)
    sources = {str(p.relative_to(snapshot)): digest(p) for p in snapshot.rglob('*') if p.is_file()}
    build, devel = output / 'private_build', output / 'private_devel'
    command = ['cmake', '-S', str(snapshot), '-B', str(build),
               '-DCATKIN_DEVEL_PREFIX=' + str(devel), '-DCATKIN_ENABLE_TESTING=OFF',
               '-DCMAKE_BUILD_TYPE=Release', '-DPYTHON_EXECUTABLE=/usr/bin/python3']
    with (output / 'build.log').open('x') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(['cmake', '--build', str(build), '--target', 'ekf_multi_object_tracking', '-j2'],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    binary = devel / 'lib/ekf_multi_object_tracking/ekf_multi_object_tracking'
    save(output / 'build_receipt.json', dict(command=command, binary=str(binary),
         binary_sha256=digest(binary), source_sha256=sources, installed=False))
    return binary


def worker(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    if port == 11311:
        raise ValueError('Refuse production port')
    env = dict(os.environ, ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
               ROS_NAMESPACE='/', ROS_HOME=str(output/'ros_home'), ROS_LOG_DIR=str(output/'roslogs'))
    env.pop('ROS_HOSTNAME', None)
    env['PYTHONPATH'] = str(PACKAGE/'src') + os.pathsep + env.get('PYTHONPATH', '')
    os.environ.update(env)
    os.environ.pop('ROS_HOSTNAME', None)
    sys.path.insert(0, str(PACKAGE/'src'))
    import rospy
    import rosbag
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Header, String
    from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects, PredictedObjects
    from perception_object_pipeline.detected_objects import make_detected_objects

    processes, handles, subscribers = [], [], []
    bag = None
    rows = {key: [] for key in ('measurement', 'policy', 'predicted')}
    diagnostics, wall_latencies = [], []
    output_by_key = {key: {} for key in rows}
    diag_by_seq, tick_started = {}, {}
    input_times = {}
    current_tick_ros_now = [None]
    tick_started_by_output_ns = {}
    wall_tick_lateness_ms = []
    failures = []
    lock = threading.RLock()
    def spawn(name, command):
        handle = (output/(name+'.log')).open('x'); handles.append(handle)
        process = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process
    started = time.monotonic()
    try:
        spawn('master', ['roscore', '-p', str(port)])
        proxy = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        def online():
            try:
                return proxy.getPid('/shape_replay')[0] == 1
            except (OSError, ConnectionError):
                return False
        wait(online, 15, 'Private master', processes)
        rospy.init_node('shape_replay', disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        configuration = yaml.safe_load((args.config or TRACKER/'config/config.yaml').read_text())
        configuration['configure'].update(fixed_output_rate_hz=30.,
            maximum_fixed_output_prediction_age_sec=.5,
            pedestrian_gaussian_association_ranking=True,
            shape_consistent_association_ranking=args.policy == 'shape_on',
            guard_inconsistent_class_transitions=False)
        if getattr(args, 'hold_unknown_output_pose', False):
            configuration['configure']['hold_unknown_output_pose'] = True
        if getattr(args, 'stationary_unknown_prediction', False):
            configuration['configure']['stationary_unknown_prediction'] = True
        map_file = getattr(args, 'map_file', None) or ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm'
        for key, value in configuration.items():
            rospy.set_param('/'+key, value)
        start, frames, excluded = load_frames(args.world_cache, args.limit)
        if args.stop_relative:
            frames = [frame for frame in frames if frame['receipt'] <= start + args.stop_relative]
        schedule = grid(frames, clock_mode=args.clock_mode)
        clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True, tcp_nodelay=True)
        publisher = rospy.Publisher(TOPICS['detection'], DetectedObjects, queue_size=10, tcp_nodelay=True)
        bag = rosbag.Bag(str(output/'replay.bag'), 'w', compression='lz4')
        def receive(message, key):
            try:
                received = time.monotonic()
                token = message.header.stamp.to_nsec()
                with lock:
                    if token in output_by_key[key]:
                        raise ValueError('Duplicate scheduled stamp '+key)
                    row = dict(stamp_ns=token, receipt_ns=rospy.Time.from_sec(current_tick_ros_now[0]).to_nsec(),
                        frame=message.header.frame_id,
                        wire_sha256=hashlib.sha256(wire(message)).hexdigest(),
                        objects=[object_row(obj, key == 'predicted') for obj in message.objects])
                    rows[key].append(row); output_by_key[key][token] = row
                    bag.write(TOPICS[key], message, t=rospy.Time.from_sec(current_tick_ros_now[0]))
                    if tick_started:
                        matched_start = next((value for target, value in reversed(tick_started_by_output_ns.items())
                                              if abs(target-token) < 1000), next(reversed(tick_started.values())))
                        wall_latencies.append(dict(stage=key, stamp_ns=token,
                            clock_to_receipt_ms=(received-matched_start)*1000))
            except Exception as error:
                failures.append(repr(error))
        def association(message):
            try:
                row = json.loads(message.data)
                with lock:
                    diagnostics.append(row); diag_by_seq[row['input_seq']] = row
                    bag.write(TOPICS['association'], message,
                              t=rospy.Time.from_sec(row['publication_stamp_sec']))
            except Exception as error:
                failures.append(repr(error))
        for key in rows:
            subscribers.append(rospy.Subscriber(TOPICS[key], PredictedObjects if key == 'predicted' else TrackedObjects,
                receive, callback_args=key, queue_size=100, tcp_nodelay=True))
        subscribers.append(rospy.Subscriber(TOPICS['association'], String, association, queue_size=20, tcp_nodelay=True))
        clock.publish(Clock(rospy.Time.from_sec(frames[0]['stamp']-.2)))
        spawn('tracker', [str(args.binary.resolve()), '__name:=shape_tracker', '_world_frame_id:=map',
            '_publish_association_diagnostics:=true',
            '~input/objects:='+TOPICS['detection'], '~output/objects:='+TOPICS['measurement'],
            '~diagnostics/association:='+TOPICS['association']])
        spawn('motion_policy', ['/usr/bin/python3', str(PACKAGE/'scripts/tracked_object_motion_policy_node.py'),
            '__name:=shape_policy', '_force_unknown_stationary:=true',
            '~input/objects:='+TOPICS['measurement'], '~output/objects:='+TOPICS['policy']])
        spawn('map_cv', ['/usr/bin/python3', str(PACKAGE/'scripts/map_based_prediction_node.py'),
            '__name:=shape_map_cv', '_prediction_horizon:=3.0', '_prediction_time_step:=0.1',
            '_mixed_horizon_enabled:=false',
            '_map_file:='+str(map_file),
            '_projector_file:='+str(ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml'),
            '~input/objects:='+TOPICS['policy'], '~output/objects:='+TOPICS['predicted']])
        wait(lambda: publisher.get_num_connections() > 0 and
             all(sub.get_num_connections() > 0 for sub in subscribers), 30, 'All stage connections', processes)
        # Source C++ uses ros::Rate(100), so advance by >20ms every tick.
        # Frozen-clock prequeue lets AsyncSpinner commit the full packet before
        # the main-loop wake. Its input-generation ACK detects any dropped input.
        time.sleep(.1)
        tracker_uri = proxy.lookupNode('/shape_replay', '/shape_tracker')[2]
        tracker_api = xmlrpc.client.ServerProxy(tracker_uri)
        def received_bytes():
            response = tracker_api.getBusStats('/shape_replay')
            # roscpp exposes this method as the bare [publish, subscribe]
            # pair, unlike its conventional [status, message, value] methods.
            if len(response) == 2 and isinstance(response[0], list):
                subscription_stats = response[1]
            elif response[0] == 1 and len(response[2]) >= 2:
                subscription_stats = response[2][1]
            else:
                raise RuntimeError('Unsupported native bus stats: '+repr(response))
            return sum(connection[1] for topic, connections in subscription_stats
                       if topic == TOPICS['detection'] for connection in connections)
        previous_received_bytes = received_bytes()
        native_output_target = schedule[0][0]
        wall_anchor = None
        wall_finished = False
        for index, (now, selected) in enumerate(schedule):
            realtime = args.wall_window and args.wall_window[0] <= now-start <= args.wall_window[1]
            if realtime:
                if wall_anchor is None:
                    wall_anchor = (now, time.monotonic()+.05)
                target_wall = wall_anchor[1] + now - wall_anchor[0]
                time.sleep(max(0., target_wall-.006-time.monotonic()))
            elif wall_anchor is not None and not wall_finished:
                time.sleep(.5)
                wall_finished = True
            for seq in selected:
                frame = frames[seq]; objects = frame['world_objects']
                message = make_detected_objects(Header(seq=seq, stamp=rospy.Time.from_sec(frame['stamp']), frame_id='map'),
                    [obj['center']+obj['size']+[obj['yaw']] for obj in objects],
                    [obj['score'] for obj in objects], [obj['class_name'] for obj in objects])
                for source_object, ros_object in zip(objects, message.objects):
                    if 'label' in source_object:
                        ros_object.classification[0].label = source_object['label']
                input_times[seq] = time.monotonic()
                publisher.publish(message)
                with lock:
                    bag.write(TOPICS['detection'], message, t=rospy.Time.from_sec(now))
                # Verify native TCP receipt while simulated time is frozen.
                # A fixed wall sleep alone failed once when transport delivery
                # crossed the main-loop wake, so it is not our receipt oracle.
                wait(lambda: received_bytes() > previous_received_bytes, 3., 'Native input TCP receipt', processes)
                previous_received_bytes = received_bytes()
                time.sleep(.003)
            if realtime:
                time.sleep(max(0., target_wall-time.monotonic()))
                wall_tick_lateness_ms.append(max(0., time.monotonic()-target_wall)*1000)
            tick_started[index] = time.monotonic()
            current_tick_ros_now[0] = now
            tick_started_by_output_ns[round(native_output_target*1.e9)] = tick_started[index]
            clock.publish(Clock(rospy.Time.from_sec(now)))
            with lock:
                bag.write('/clock', Clock(rospy.Time.from_sec(now)), t=rospy.Time.from_sec(now))
            # The actual C++ scheduler accumulates a double period rather
            # than recomputing first+tick/30. At epoch-scale ROS timestamps
            # this drifts by ~32ns/tick, deterministically, relative to /clock.
            expected_ns = round(native_output_target*1.e9)
            def acknowledged():
                if failures:
                    raise RuntimeError(failures)
                with lock:
                    return (all(seq+1 in diag_by_seq for seq in selected) and
                            all(any(abs(token-expected_ns) < 1000 for token in outputs)
                                for outputs in output_by_key.values()))
            if not realtime:
                wait(acknowledged, 3., 'Tick ACK '+str(index)+' inputs='+str(selected), processes)
            # Allow the native loop to enter its next ros::Rate sleep; clock
            # remains frozen and no additional scheduled output can appear.
            time.sleep(.001)
            native_output_target += 1./30.
            if index % 1000 == 0:
                print(json.dumps(dict(policy=args.policy, tick=index, inputs=len(diag_by_seq))), flush=True)
        with lock:
            # rospy.Publisher fills the serialized Header.seq from its own
            # connection-wide counter, starting at one (source index+1).
            missing_input_sequences = sorted(set(range(1, len(frames)+1))-set(diag_by_seq))
            if not args.wall_window and (len(diagnostics) != len(frames) or missing_input_sequences):
                raise ValueError('Input receipt sequence mismatch')
            if not args.wall_window and [d['input_generation'] for d in diagnostics] != list(range(1, len(frames)+1)):
                raise ValueError('Input generation mismatch')
            if any(d['truncated'] for d in diagnostics):
                raise ValueError('Association diagnostics truncated')
            for key in rows:
                rows[key].sort(key=lambda row: row['stamp_ns'])
                if not args.wall_window and len(rows[key]) != len(schedule):
                    raise ValueError('Output coverage mismatch '+key)
            for row in rows['predicted']:
                for obj in row['objects']:
                    if any(not valid_three_second_path(p, getattr(args, 'allow_shorter_map_paths', False))
                           for p in obj['paths']):
                        raise ValueError('3 second prediction contract changed')
            with gzip.open(output/'rows.json.gz', 'xt') as handle:
                json.dump(rows, handle, allow_nan=False)
            with gzip.open(output/'association.json.gz', 'xt') as handle:
                json.dump(diagnostics, handle, allow_nan=False)
            events = Counter(d['event'] for row in diagnostics for d in row['rows'])
            births = [dict(seq=update['input_seq'], measurement_stamp=update['measurement_stamp_sec'], **r)
                      for update in diagnostics for r in update['rows'] if r['event'] == 'birth']
            confirms = {}
            for update in diagnostics:
                for row in update['rows']:
                    if row['confirmed'] and row['track_uuid'] not in confirms:
                        confirms[row['track_uuid']] = dict(stamp=update['measurement_stamp_sec'],
                            since_birth_sec=update['measurement_stamp_sec']-row['initialization_stamp_sec'])
            save(output/'summary.json', dict(policy=args.policy, master_uri=env['ROS_MASTER_URI'],
                binary_sha256=digest(args.binary), input_cache_sha256=digest(args.world_cache),
                map_file=str(map_file.resolve()), map_sha256=digest(map_file),
                allow_shorter_map_paths=getattr(args, 'allow_shorter_map_paths', False),
                source_sha256={str(p):digest(p) for p in [Path(__file__).resolve(),
                    PACKAGE/'scripts/map_based_prediction_node.py',
                    PACKAGE/'scripts/tracked_object_motion_policy_node.py',
                    *sorted((PACKAGE/'src/perception_object_pipeline').glob('*.py')),
                    map_file,
                    ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml']},
                input_state_fingerprint=fingerprint(frames), schedule_fingerprint=fingerprint(schedule),
                clock_mode=args.clock_mode,
                wall_paced_window_relative_sec=args.wall_window,
                wall_tick_lateness_ms=stats(wall_tick_lateness_ms),
                diagnostic_receipts=len(diagnostics), missing_input_sequences=missing_input_sequences,
                expected_output_count=len(schedule),
                output_gap_sec={k:stats([(b['stamp_ns']-a['stamp_ns'])*1.e-9 for a,b in zip(v,v[1:])]) for k,v in rows.items()},
                bag_start=start, excluded_tf_stamps=excluded, input_count=len(frames),
                output_counts={k:len(v) for k,v in rows.items()},
                payload_fingerprints={k:fingerprint(v) for k,v in rows.items()},
                diagnostic_fingerprint=fingerprint(diagnostics), events=dict(events), births=births,
                unique_output_uuids={k:len({o['uuid'] for row in v for o in row['objects']}) for k,v in rows.items()},
                confirmation_delay_sec=stats([v['since_birth_sec'] for v in confirms.values()]),
                input_header_to_sim_current_sec=stats([d['publication_stamp_sec']-d['measurement_stamp_sec'] for d in diagnostics]),
                stage_clock_to_wall_receipt_ms={k:stats([v['clock_to_receipt_ms'] for v in wall_latencies if v['stage']==k]) for k in rows},
                effective_configure=configuration['configure'], wall_elapsed_sec=time.monotonic()-started,
                clock_contract=('Recorded-TF world-cache reconstructed messages retain cached float measurement time; current time 30Hz grid anchored first recorded input receipt; each input appears at first tick >= original receipt.' if args.clock_mode == 'receipt' else
                                'Recorded-TF world-cache reconstructed messages retain cached float measurement time; synthetic current time 30Hz grid starting first measurement+.020s.') +
                    (' Selected wall window is 1x, downstream ACK-nonblocking; other history is ACK-stepped.' if args.wall_window else ' ACK-paced, not wall-rate runtime.') +
                    ' Output bag receipt is callback current simulated time, distinct from header and measurement time. Header frame is map and seq is regenerated.',
                production_modified=False, lan_closed_loop_certified=False))
    finally:
        for sub in subscribers:
            sub.unregister()
        for process in reversed(processes):
            stop(process)
        if bag is not None:
            bag.close()
        for handle in handles:
            handle.close()
        process_table = subprocess.check_output(['ps','-eo','pid=,pgid=,args='], text=True).splitlines()
        owned_groups = {p.pid for p in processes}
        remaining = [line.strip() for line in process_table if int(line.split()[1]) in owned_groups]
        with socket.socket() as check:
            check.settimeout(.2)
            master_port_closed = check.connect_ex(('127.0.0.1', port)) != 0
        save(output/'cleanup.json', dict(owned_pids=[p.pid for p in processes],
            exited=[p.poll() is not None for p in processes], owned_process_groups_remaining=remaining,
            private_master_port_closed=master_port_closed, production_touched=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--world-cache', type=Path, default=DEFAULT_CACHE)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--map-file', type=Path,
                        help='Explicit map for a new replay; historical default remains v85')
    parser.add_argument('--hold-unknown-output-pose', action='store_true',
                        help='Opt-in stationary UNKNOWN output-copy candidate; no internal EKF changes')
    parser.add_argument('--stationary-unknown-prediction', action='store_true',
                        help='Opt-in stationary UNKNOWN internal transition candidate')
    parser.add_argument('--allow-shorter-map-paths', action='store_true',
                        help='Accept current road-end truncation within the same 3 second cap')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--policy', choices=('off_a','off_b','shape_on'))
    parser.add_argument('--clock-mode', choices=('receipt','measurement'), default='receipt')
    parser.add_argument('--wall-window', nargs=2, type=float, metavar=('START','END'))
    parser.add_argument('--stop-relative', type=float)
    args = parser.parse_args()
    if args.wall_window and (not args.worker or not 0 <= args.wall_window[0] < args.wall_window[1]):
        parser.error('--wall-window requires a worker and an increasing nonnegative window')
    if args.worker and (args.binary is None or args.policy is None):
        parser.error('--worker requires --binary and --policy')
    if args.worker:
        return worker(args)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    binary = args.binary or private_build(args.output)
    summaries = {}
    for policy in ('off_a', 'off_b', 'shape_on'):
        command = [sys.executable, str(Path(__file__).resolve()), '--worker', '--policy', policy,
                   '--output', str(args.output/policy), '--binary', str(binary.resolve()),
                   '--world-cache', str(args.world_cache.resolve())]
        command += ['--clock-mode', args.clock_mode]
        if args.limit:
            command += ['--limit', str(args.limit)]
        if args.config:
            command += ['--config', str(args.config.resolve())]
        if args.map_file:
            command += ['--map-file', str(args.map_file.resolve())]
        if args.hold_unknown_output_pose:
            command += ['--hold-unknown-output-pose']
        if args.stationary_unknown_prediction:
            command += ['--stationary-unknown-prediction']
        if args.allow_shorter_map_paths:
            command += ['--allow-shorter-map-paths']
        if args.stop_relative:
            command += ['--stop-relative', str(args.stop_relative)]
        subprocess.run(command, check=True)
        summaries[policy] = json.loads((args.output/policy/'summary.json').read_text())
        if policy == 'off_b':
            for field in ('input_state_fingerprint', 'schedule_fingerprint', 'payload_fingerprints', 'diagnostic_fingerprint'):
                if summaries['off_a'][field] != summaries['off_b'][field]:
                    raise ValueError('OFF repeated replay differs in '+field+'; shape run blocked')
    report = dict(complete=True, off_repeat_exact=True, production_modified=False,
                  lan_closed_loop_certified=False, policies=summaries,
                  command=sys.argv, topics=TOPICS,
                  limitations=['World-frame cached inputs bypass recorded-TF lookup in this run.',
                    'ACK-stepped simulated time measures functional ROS transport/scheduler, not 30Hz wall-rate latency or live input backlog.',
                    'No fresh raw sensor, detector inference, LAN timing or closed-loop planner safety certification.'])
    save(args.output/'report.json', report)
    print(json.dumps(dict(complete=True, off_repeat_exact=True, output=str(args.output))), flush=True)


if __name__ == '__main__':
    main()
