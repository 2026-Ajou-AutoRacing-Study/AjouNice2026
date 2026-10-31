#!/usr/bin/env python3
"""Bounded development-only route oracle and timestamped ROS trial recorder.

Publishes ONLY UInt64 trial route IDs. Ego is simulator development GT, never
localization input/certification. No vehicle command, TF, RPC, JPEG or subprocess.
ROS imports are lazy so pure tests need no ROS installation.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import struct
import threading
import time

MISSION_IDS = frozenset((201351, 201365, 201290, 201326))
ROUTE_TOPIC = '/traffic_light_trial/route_group_id'
ALLOWED_TYPES = frozenset(('std_msgs/String', 'nav_msgs/Odometry', 'sensor_msgs/Imu',
    'morai_msgs/GPSMessage', 'autoware_perception_msgs/TrafficLightGroupArray'))


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Finite numeric coordinate required')
    return float(value)


def load_plan(path):
    path = Path(path).resolve(strict=True)
    payload = path.read_bytes()
    document = json.loads(payload)
    rows = document.get('poses')
    if not isinstance(rows, list) or not 1 <= len(rows) <= 64:
        raise ValueError('Explicit plan.poses list (1..64) required')
    result = []
    for row in rows:
        group = row.get('group_regulatory_id')
        if type(group) is not int or group not in MISSION_IDS:
            raise ValueError('Only A/B/C/D2 mission regulatory IDs are allowed')
        pose = row.get('pose', {})
        name = row.get('group_id')
        if not isinstance(name, str) or not name or len(name) > 200:
            raise ValueError('Explicit pose group_id required')
        result.append({'group_id': name, 'group_regulatory_id': group,
                       'x': finite(pose.get('x')), 'y': finite(pose.get('y')),
                       'yaw': finite(pose.get('yaw'))})
    return result, {'path': str(path), 'sha256': hashlib.sha256(payload).hexdigest(),
                    'poses': result, 'source_document': document}


def yaw_distance(a, b):
    return abs((a - b + 180.) % 360. - 180.)


def select_group(poses, ego, now_monotonic):
    if ego is None:
        return {'group_id': 0, 'reason': 'ego_missing'}
    try:
        age = finite(now_monotonic) - finite(ego['receipt_monotonic'])
        x, y, yaw = (finite(ego[k]) for k in ('x', 'y', 'heading'))
    except (KeyError, TypeError, ValueError):
        return {'group_id': 0, 'reason': 'ego_invalid'}
    if not 0 <= age < .5:
        return {'group_id': 0, 'reason': 'ego_receipt_stale_or_clock_regressed', 'ego_age_sec': age}
    candidates = []
    for pose in poses:
        distance, angle = math.hypot(x - pose['x'], y - pose['y']), yaw_distance(yaw, pose['yaw'])
        if distance < 2. and angle < 15.:
            candidates.append({'group_id': pose['group_regulatory_id'], 'pose_group_id': pose['group_id'],
                               'xy_distance_m': distance, 'yaw_distance_degrees': angle})
    if not candidates:
        return {'group_id': 0, 'reason': 'outside_explicit_plan_pose_gate', 'ego_age_sec': age}
    if len({row['group_id'] for row in candidates}) != 1:
        return {'group_id': 0, 'reason': 'ambiguous_mission_groups', 'candidates': candidates,
                'ego_age_sec': age}
    return {'group_id': candidates[0]['group_id'], 'reason': 'development_ego_pose_match',
            'ego_age_sec': age, 'matches': candidates}


def stamp_ns(stamp):
    if stamp is None:
        return None
    if hasattr(stamp, 'secs') and hasattr(stamp, 'nsecs'):
        return int(stamp.secs) * 1000000000 + int(stamp.nsecs)
    return None


def message_stamp_ns(stamp):
    """Strict builtin_interfaces/Time representation; zero is a sentinel."""
    if stamp is None:
        return None
    sec, nanosec = getattr(stamp, 'sec', None), getattr(stamp, 'nanosec', None)
    if (type(sec) is not int or type(nanosec) is not int
            or sec < 0 or not 0 <= nanosec < 1000000000):
        return None
    return sec * 1000000000 + nanosec


def timestamp_contracts():
    return {
        'units': 'integer_nanoseconds',
        'source_header_stamp_ns': 'actual message.header.stamp (ROS1 secs/nsecs) only',
        'source_message_stamp_ns': 'actual top-level message.stamp (builtin_interfaces sec/nanosec) only; zero sentinel is not an image association',
        'image_association_priority': ['positive_valid_message.stamp', 'positive_valid_parsed_debug.source_stamp_ns'],
        'association_rule': 'Exact source-stamp equality in a verified common ROS clock; never nearest/equal receipt time. Generic odometry/IMU header stamps are not automatically camera-image IDs.',
        'source_stamp_clock': 'publisher ROS source clock; epoch and use_sim_time must be checked externally',
        'receipt_ros_ns_clock': 'rospy.Time.now ROS clock, not independently asserted UTC',
        'receipt_monotonic_clock': 'local host time.monotonic; never numerically compared or exact-joined with ROS stamps',
        'common_source_and_receipt_ros_clock_independently_verified': False,
    }


def to_json_value(value, depth=0):
    """Serialize only declared ROS fields; never arbitrary object attributes."""
    if depth > 12:
        raise ValueError('Message nesting limit')
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else {'nonfinite': repr(value)}
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {'bytes_omitted': len(value), 'reason': 'binary_payload_not_recorded'}
    if isinstance(value, (list, tuple)):
        if len(value) > 1024:
            raise ValueError('Message array length limit')
        return [to_json_value(item, depth + 1) for item in value]
    if isinstance(value, dict):
        if len(value) > 1024 or any(not isinstance(key, str) for key in value):
            raise ValueError('Message dictionary limit or non-string key')
        return {key: to_json_value(item, depth + 1) for key, item in value.items()}
    slots = getattr(value, '__slots__', None)
    if not slots:
        raise ValueError('Unsupported message object')
    return {key: to_json_value(getattr(value, key), depth + 1)
            for key in slots if not key.startswith('_')}


def compressed_header(payload):
    """Read ROS1 Header only; do not deserialize/copy/decode compressed JPEG."""
    if len(payload) < 16:
        raise ValueError('Truncated compressed-image header')
    seq, secs, nsecs, length = struct.unpack_from('<IIII', payload, 0)
    if nsecs >= 1000000000 or length > 4096 or 16 + length > len(payload):
        raise ValueError('Invalid compressed-image header')
    frame = bytes(memoryview(payload)[16:16 + length]).decode('utf-8', errors='strict')
    return {'seq': seq, 'stamp': {'secs': secs, 'nsecs': nsecs}, 'frame_id': frame}, secs*1000000000+nsecs


def observation_record(topic, message, receipt_ros_ns, receipt_monotonic):
    started = time.monotonic()
    header = getattr(message, 'header', None)
    source_stamp = stamp_ns(getattr(header, 'stamp', None))
    top_stamp = message_stamp_ns(getattr(message, 'stamp', None))
    record = {'event': 'observation', 'topic': topic, 'receipt_ros_ns': receipt_ros_ns,
              'receipt_monotonic': receipt_monotonic, 'source_header_stamp_ns': source_stamp,
              'source_message_stamp_ns': top_stamp,
              'image_association_stamp_ns': top_stamp if top_stamp is not None and top_stamp > 0 else None,
              'image_association_stamp_origin': 'message.stamp' if top_stamp is not None and top_stamp > 0 else None,
              'message_type': getattr(message, '_type', None), 'message': to_json_value(message)}
    if getattr(message, '_type', None) == 'std_msgs/String':
        raw = message.data
        if len(raw.encode('utf-8')) > 262144:
            raise ValueError('String exceeds 256KiB observation limit')
        # Preserve original text. Parsed debug metadata can independently carry
        # image/source/generated stamps; receipt equality is NEVER association.
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                record['parsed_debug'] = to_json_value(parsed)
                debug_stamp = parsed.get('source_stamp_ns')
                if type(debug_stamp) is int and debug_stamp > 0:
                    if record['image_association_stamp_ns'] is None:
                        record['image_association_stamp_ns'] = debug_stamp
                        record['image_association_stamp_origin'] = 'parsed_debug.source_stamp_ns'
                    elif debug_stamp != record['image_association_stamp_ns']:
                        record['image_association_stamp_conflict'] = True
        except (ValueError, TypeError):
            pass
    record['observer_serialization_sec'] = time.monotonic() - started
    record['not_node_compute_time'] = True
    return record


class JsonlWriter:
    def __init__(self, path):
        self.stream = Path(path).open('x', encoding='utf-8')
        self.lock = threading.Lock()
        self.counts = Counter()
        self.closed = False
    def write(self, record):
        with self.lock:
            if self.closed:
                return
            self.stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
            self.stream.flush()
            self.counts[record.get('topic', record.get('event', 'unknown'))] += 1
    def close(self):
        with self.lock:
            if not self.closed:
                self.stream.flush(); self.stream.close(); self.closed = True


def validate_route_topic(value):
    if not re.fullmatch(r'/traffic_light_trial/[A-Za-z0-9_/]+', value):
        raise ValueError('Route publisher must stay under /traffic_light_trial/')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=1800.)
    parser.add_argument('--route-topic', default=ROUTE_TOPIC)
    parser.add_argument('--node-namespace', default='/traffic_light_trial/recognizer')
    parser.add_argument('--camera-headers', action='store_true')
    args = parser.parse_args(argv)
    poses, plan = load_plan(args.plan)
    validate_route_topic(args.route_topic)
    if not re.fullmatch(r'/traffic_light_trial/[A-Za-z0-9_/]+', args.node_namespace):
        parser.error('Node namespace must stay below /traffic_light_trial/')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 7200:
        parser.error('Duration must be finite, >0 and <=7200 seconds')
    output = args.output.expanduser().absolute()
    if output.exists() or output.is_symlink():
        parser.error('Output directory must be fresh')
    import rospy
    from roslib.message import get_message_class
    from morai_msgs.msg import EgoVehicleStatus
    from std_msgs.msg import UInt64
    rospy.init_node('traffic_light_trial_monitor', anonymous=True, disable_signals=True)
    output.mkdir(parents=True, exist_ok=False)
    metadata = {'development_only': True, 'plan': plan, 'duration_sec': args.duration,
       'started_utc': datetime.now(timezone.utc).isoformat(), 'only_publisher': args.route_topic,
       'route_message_type': 'std_msgs/UInt64', 'route_rate_hz': 10,
       'route_authority': 'SIMULATOR_EGO_DEVELOPMENT_GT_NOT_OPERATIONAL_ROUTE_CERTIFICATION',
       'ego_usage': 'pose-gated trial route oracle only; NEVER localization/TF input',
       'gate': {'xy_strictly_below_m': 2., 'yaw_strictly_below_degrees': 15.,
                'ego_receipt_monotonic_age_strictly_below_sec': .5,
                'yaw_reference': 'plan scenario yaw degrees directly versus Ego heading degrees; no heading correction'},
       'jpeg_recording': False, 'camera_headers_enabled': args.camera_headers,
       'observation_rate_caps_hz': {'debug_and_map_and_signals_and_odometry_and_GPS':10,
                                    'IMU':5,'optional_camera_headers':30},
       'observation_type_allowlist': sorted(ALLOWED_TYPES),
       'timestamp_contracts': timestamp_contracts(),
       'association_rule': 'Use exact source image stamp fields, never nearest receipt time as proven association.',
       'timing_caveat': 'observer_serialization_sec is recorder overhead, not node compute latency. Debug source/generated stamps and node-reported timings are separate evidence.',
       'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    with (output/'metadata.json').open('x') as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2, allow_nan=False)
    writer = JsonlWriter(output/'data.jsonl')
    stop = threading.Event(); lock = threading.Lock(); latest = [None]
    subscriptions = []; last = {}; throttled = Counter(); errors = Counter()
    publisher = rospy.Publisher(args.route_topic, UInt64, queue_size=1, latch=False)
    def stop_signal(signum, frame):
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, stop_signal)
    def observe(topic, message, ros_ns, mono):
        try: writer.write(observation_record(topic, message, ros_ns, mono))
        except Exception as error:
            errors[topic] += 1
            writer.write({'event':'observation_error','topic':topic,'receipt_ros_ns':ros_ns,
                          'receipt_monotonic':mono,'error':type(error).__name__+':'+str(error)})
    def ego_callback(message):
        mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
        with lock:
            latest[0] = {'x':message.position.x,'y':message.position.y,'heading':message.heading,
                         'receipt_monotonic':mono,'receipt_ros_ns':ros_ns,
                         'source_header_stamp_ns':stamp_ns(getattr(getattr(message,'header',None),'stamp',None))}
        if mono-last.get('/Ego_topic',float('-inf')) >= .1:
            last['/Ego_topic']=mono; observe('/Ego_topic',message,ros_ns,mono)
    subscriptions.append(rospy.Subscriber('/Ego_topic', EgoVehicleStatus, ego_callback, queue_size=1))
    def callback_for(topic, hz, camera=False):
        def callback(raw):
            mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
            if mono-last.get(topic,float('-inf')) < 1./hz:
                throttled[topic] += 1; return
            last[topic] = mono
            try:
                msgtype=raw._connection_header.get('type')
                if camera:
                    if msgtype != 'sensor_msgs/CompressedImage': raise ValueError('Unexpected camera type')
                    header, stamp = compressed_header(raw._buff)
                    writer.write({'event':'camera_header','topic':topic,'receipt_ros_ns':ros_ns,
                        'receipt_monotonic':mono,'source_header_stamp_ns':stamp,'header':header,
                        'serialized_message_bytes':len(raw._buff),'JPEG_bytes_not_saved':True})
                else:
                    if msgtype not in ALLOWED_TYPES: raise ValueError('Unexpected topic message type')
                    cls=get_message_class(msgtype)
                    if cls is None: raise ValueError('ROS message class unavailable')
                    observe(topic,cls().deserialize(raw._buff),ros_ns,mono)
            except Exception as error:
                errors[topic] += 1
                writer.write({'event':'decode_error','topic':topic,'receipt_ros_ns':ros_ns,
                              'receipt_monotonic':mono,'error':type(error).__name__+':'+str(error)})
        return callback
    topics = {args.node_namespace+'/'+name:10. for name in
        ('debug_selection','debug_classifier','debug_policy_reason','debug_observed_label','debug_raw_label',
         'debug_runtime')}
    topics.update({'/traffic_light_trial/map_rois':10.,'/traffic_light_trial/traffic_signals':10.,
                   '/localization/kinematic_state':10.,'/sensor/imu':5.,'/sensor/gps':10.})
    if args.camera_headers: topics['/sensor/camera/front/compressed']=30.
    for topic,hz in topics.items():
        subscriptions.append(rospy.Subscriber(topic,rospy.AnyMsg,callback_for(topic,hz,topic.endswith('/compressed')),
                                               queue_size=3,buff_size=2**22))
    start=time.monotonic(); next_route=start; status='duration_complete'
    try:
        while not stop.is_set() and not rospy.is_shutdown() and time.monotonic()-start < args.duration:
            mono=time.monotonic()
            with lock: ego=dict(latest[0]) if latest[0] is not None else None
            choice=select_group(poses,ego,mono)
            publisher.publish(UInt64(data=choice['group_id']))
            writer.write({'event':'route_feed','topic':args.route_topic,'receipt_ros_ns':rospy.Time.now().to_nsec(),
                          'receipt_monotonic':mono,'development_GT_only':True,'selection':choice,'ego_snapshot':ego})
            next_route += .1
            now=time.monotonic()
            if next_route < now: next_route=now+.1
            stop.wait(min(max(0.,next_route-now),max(0.,args.duration-(now-start))))
        if stop.is_set():status='signal_stop'
        elif rospy.is_shutdown():status='ROS_shutdown'
    except BaseException:
        status='failed';raise
    finally:
        for sub in subscriptions:
            try: sub.unregister()
            except Exception: errors['own_subscription_cleanup'] += 1
        # Withdraw only this process's trial route lease; never touch other nodes.
        try: publisher.publish(UInt64(data=0))
        except Exception: pass
        try: publisher.unregister()
        except Exception: errors['own_publisher_cleanup'] += 1
        writer.close()
        with (output/'summary.json').open('x') as stream:
            json.dump({'status':status,'elapsed_monotonic_sec':time.monotonic()-start,
              'records_by_topic':dict(writer.counts),'throttled_by_topic':dict(throttled),
              'errors_by_topic':dict(errors),'only_own_subscriptions_and_publisher_closed':True},stream,indent=2)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
