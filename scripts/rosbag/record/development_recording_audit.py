"""Offline recording completeness/timestamps, not a driving-safety verdict."""
from collections import Counter
import json
import math
import re
import struct


CAMERAS = ['/sensor/camera/' + n + '/compressed'
           for n in ('front', 'front_left', 'front_right', 'back')]
DET = '/perception/object_recognition/detection/objects'
MEASUREMENT = '/perception/object_recognition/tracking/measurement_objects'
TRACK = '/perception/object_recognition/tracking/objects'
PRED = '/perception/object_recognition/objects'
TIMINGS = ['/perception/' + n + '/frame_timing' for n in ('bevfusion', 'base5_trial')]
GROUPS = {
    'raw_sensor_replay': {'all': ['/sensor/lidar1', '/sensor/lidar1/packets'] + CAMERAS},
    'sensor_localization': {'all': ['/sensor/gps', '/sensor/imu', '/Competition_topic',
                                   '/localization/kinematic_state', '/tf', '/tf_static']},
    'detection_tracking_prediction': {'all': [DET, MEASUREMENT, TRACK, PRED]},
    'tracker_assignment_trace': {'all': ['/perception/object_recognition/tracking/diagnostics/association']},
    'detector_stage_timing': {'any': TIMINGS},
    'postprocessing_stage_timing': {'all': [
        '/tracked_object_motion_policy/diagnostics/processing_time_ms',
        '/map_based_prediction/diagnostics/processing_time_ms']},
    'lane_change_reason': {'any': [
        '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/'
        'debug/lane_change_' + side + '/safety_checks' for side in ('left', 'right')]},
    'cruise_reason': {'all': [
        '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/'
        'debug/obstacle_cruise/' + suffix for suffix in ('selection', 'input_trajectory')]},
    'planning_and_control': {'all': ['/planning/trajectory', '/planning/validation_status', '/ctrl_cmd']},
    'development_pose_oracle': {'all': ['/Ego_topic', '/Object_topic']},
}
# These ROS messages begin with std_msgs/Header. Never guess this for unknown types.
HEADER_TYPES = {'sensor_msgs/CompressedImage', 'sensor_msgs/PointCloud2',
                'sensor_msgs/Imu', 'sensor_msgs/NavSatFix', 'velodyne_msgs/VelodyneScan',
                'morai_msgs/GPSMessage', 'morai_msgs/EgoVehicleStatus',
                'morai_msgs/Competition', 'nav_msgs/Odometry',
                'autoware_auto_perception_msgs/DetectedObjects',
                'autoware_auto_perception_msgs/TrackedObjects',
                'autoware_auto_perception_msgs/PredictedObjects',
                'autoware_auto_planning_msgs/Trajectory',
                'autoware_perception_msgs/DetectedObjects',
                'autoware_perception_msgs/TrackedObjects',
                'autoware_perception_msgs/PredictedObjects',
                'autoware_planning_msgs/Trajectory'}
TIMED_TOPICS = set(CAMERAS + ['/sensor/lidar1', '/sensor/lidar1/packets',
    '/sensor/gps', '/sensor/imu', '/Competition_topic', '/Ego_topic',
    '/localization/kinematic_state', '/localization/control_kinematic_state',
    DET, MEASUREMENT, TRACK, PRED, '/planning/trajectory', '/ctrl_cmd'])


def summary(values):
    if not values:
        return {'count': 0}
    a = sorted(values)
    def percentile(p):
        x = (len(a) - 1) * p
        lo = int(x)
        return a[lo] + (a[min(lo + 1, len(a) - 1)] - a[lo]) * (x - lo)
    return dict(count=len(a), mean=sum(a) / len(a), minimum=a[0],
                p50=percentile(.5), p90=percentile(.9), p95=percentile(.95),
                p99=percentile(.99), maximum=a[-1])


def capabilities(topics):
    result = {}
    for name, group in GROUPS.items():
        counts = {n: topics.get(n, {}).get('messages', 0)
                  for n in group.get('all', []) + group.get('any', [])}
        absent = [n for n in group.get('all', []) if not counts[n]]
        any_ok = not group.get('any') or any(counts[n] for n in group['any'])
        result[name] = dict(recorded=not absent and any_ok,
                            missing_all_required=absent,
                            missing_any_alternative=group.get('any', []) if not any_ok else [],
                            message_counts=counts)
    return result


def raw_bytes(message):
    """Accept rosbag raw tuple layouts without deserializing image/cloud payloads."""
    value = message[1]
    return value[0] if isinstance(value, tuple) else value


def header(data):
    if len(data) < 16:
        raise ValueError('Incomplete Header')
    seq, sec, nsec, length = struct.unpack_from('<IIII', data)
    if nsec >= 1000000000 or length > len(data) - 16:
        raise ValueError('Invalid Header')
    return seq, sec * 1000000000 + nsec, 16 + length


def audit(path, patterns, required, diagnostics):
    import rosbag
    times = {}
    stage = {'messages': 0, 'invalid_json': 0, 'metrics': {}, 'last_counters_by_topic': {}}
    packet_counts = Counter()
    with rosbag.Bag(str(path)) as bag:
        topics = {k: dict(messages=v.message_count, type=v.msg_type)
                  for k, v in bag.get_type_and_topic_info().topics.items()}
        duration = bag.get_end_time() - bag.get_start_time() if topics else 0.
        selected = sorted(TIMED_TOPICS.intersection(topics))
        for topic, message, receipt in bag.read_messages(topics=selected, raw=True):
            row = times.setdefault(topic, dict(receipts=[], ages=[], seq=[], invalid_headers=0))
            row['receipts'].append(receipt.to_sec())
            if message[0] not in HEADER_TYPES:
                continue
            try:
                data = raw_bytes(message)
                seq, stamp, offset = header(data)
                row['seq'].append(seq)
                if stamp:
                    row['ages'].append((receipt.to_nsec() - stamp) / 1e6)
                if message[0] == 'velodyne_msgs/VelodyneScan':
                    packet_counts[struct.unpack_from('<I', data, offset)[0]] += 1
            except (ValueError, struct.error, TypeError):
                row['invalid_headers'] += 1
        for topic, message, _ in bag.read_messages(topics=[t for t in TIMINGS if t in topics]):
            stage['messages'] += 1
            try:
                data = json.loads(message.data)
                if not isinstance(data, dict):
                    raise ValueError('Frame timing must be an object')
                for key, value in data.items():
                    if key.endswith('_ms') and isinstance(value, (int, float)) and not isinstance(value, bool):
                        if math.isfinite(value):
                            stage['metrics'].setdefault(key, []).append(value)
                if isinstance(data.get('counters'), dict):
                    stage['last_counters_by_topic'][topic] = data['counters']
            except (ValueError, TypeError, AttributeError):
                stage['invalid_json'] += 1
    cadence = {}
    for topic, row in times.items():
        t = row['receipts']
        gaps = [(b - a) * 1000 for a, b in zip(t, t[1:])]
        sequences = list(zip(row['seq'], row['seq'][1:]))
        cadence[topic] = dict(messages=len(t),
            receipt_hz=(len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else None,
            receipt_gap_ms=summary(gaps), header_to_bag_receipt_ms=summary(row['ages']),
            negative_age_count=sum(v < 0 for v in row['ages']),
            gaps_over_250ms=sum(v > 250 for v in gaps),
            header_sequence_skips=sum(b - a - 1 for a, b in sequences if b > a + 1),
            header_sequence_nonincreasing=sum(b <= a for a, b in sequences),
            invalid_header_count=row['invalid_headers'])
    stage['metrics'] = {k: summary(v) for k, v in stage['metrics'].items()}
    missing = lambda names: [n for n in names if topics.get(n, {}).get('messages', 0) == 0]
    return dict(schema_version=2, duration_sec=duration, topics=topics,
        missing_core=missing(required), missing_diagnostics=missing(diagnostics),
        analysis_capabilities=capabilities(topics), cadence=cadence, detector_frame_timing=stage,
        lidar_packets_per_scan_histogram={str(k): v for k, v in sorted(packet_counts.items())},
        unmatched_patterns=[p for p in patterns if not any(re.match(p, t) for t in topics)],
        note='recorded=true proves message presence only, not adequate rate, accuracy, synchronization, '
             'driving safety, or complete mission coverage. Inactive LC/Cruise/GT topics may be absent. '
             'Header sequence gaps can include ROS/recorder drops or publisher restart; they are not '
             'MORAI UDP packet loss. Receiver-time headers cannot measure source exposure/physical LAN '
             'delay. Frame counters are publisher cumulative snapshots, not a session-only loss rate.')
