#!/usr/bin/env python3
"""Development-only signal compliance observer: subscriptions and native GET only.

Never starts a stack, publishes vehicle/signal commands, or changes MORAI.
Front-point crossings and native lamp bits are evaluation proxies, not official
stop-line/penalty certification. No current-state extrapolation across GT gaps.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
GROUP_IDS = {201351: ('C1256W000077',), 201365: ('C1256W000091',),
             201290: ('C1256W000016', 'C1256W000018'), 201326: ('C1256W000052',)}
HEADING_CORRECTION_DEGREES = -1.347
AGE_THRESHOLDS_SEC = (.25, .5)


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('Finite numeric value required')
    return float(value)


def load_geometry(path):
    payload = path.read_bytes()
    document = json.loads(payload)
    if document.get('frame', document.get('geometry_frame')) != 'map':
        raise ValueError('Explicit map-frame stop-line geometry required')
    groups = document.get('groups')
    if not isinstance(groups, list) or {row.get('group_id') for row in groups} != set(GROUP_IDS) or len(groups) != 4:
        raise ValueError('Exactly the four explicit mission groups are required')
    for group in groups:
        ids = group.get('native_ids')
        if not isinstance(ids, list) or set(ids) != set(GROUP_IDS[group['group_id']]) or len(ids) != len(set(ids)):
            raise ValueError('Exact mission native IDs required, including both C heads')
        points = group.get('stop_line_xy')
        if not isinstance(points, list) or not 2 <= len(points) <= 16:
            raise ValueError('Explicit stop-line polyline with 2..16 XY points required')
        for point in points:
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError('Stop-line points must be XY pairs')
            for value in point: finite(value)
        if any(math.dist(a, b) < 1e-6 for a, b in zip(points, points[1:])):
            raise ValueError('Stop-line segment must not be degenerate')
        if group.get('crossing_direction', 'both') not in ('both', 'negative_to_positive', 'positive_to_negative'):
            raise ValueError('Invalid signed stop-line crossing direction')
    return groups, {'path': str(path.resolve()), 'sha256': hashlib.sha256(payload).hexdigest(), 'document': document}


def segment_crossing(previous, current, start, end, direction='both'):
    """Directed finite-segment intersection; collinear/touch-return is not a crossing.

    Signed side is cross(end-start, point-start). Endpoint touches count once
    when arriving from a nonzero side, not again when leaving that endpoint.
    """
    previous, current, start, end = [tuple(map(finite, point)) for point in (previous, current, start, end)]
    if any(len(point) != 2 for point in (previous, current, start, end)):
        raise ValueError('XY points required')
    if direction not in ('both', 'negative_to_positive', 'positive_to_negative'):
        raise ValueError('Invalid direction')
    vx, vy = end[0]-start[0], end[1]-start[1]
    length2 = vx*vx+vy*vy
    if length2 < 1e-12:
        raise ValueError('Degenerate stop line')
    side = lambda point: vx*(point[1]-start[1])-vy*(point[0]-start[0])
    before, after = side(previous), side(current)
    # A source point already on the line has no known incoming side.
    if abs(before) <= 1e-9 or before*after > 0 or abs(before-after) <= 1e-12:
        return None
    observed_direction = 'negative_to_positive' if before < 0 else 'positive_to_negative'
    if direction != 'both' and direction != observed_direction:
        return None
    fraction = before/(before-after)
    if not 0 <= fraction <= 1:
        return None
    point = [previous[index]+fraction*(current[index]-previous[index]) for index in (0, 1)]
    along = ((point[0]-start[0])*vx+(point[1]-start[1])*vy)/length2
    if not -1e-9 <= along <= 1+1e-9:
        return None
    return {'point_xy_m': point, 'motion_fraction': fraction,
            'stop_segment_fraction': along, 'direction': observed_direction,
            'direction_configured': direction != 'both'}


def rotate(vector, quaternion):
    x, y, z, w = map(finite, quaternion)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = (value/norm for value in (x, y, z, w))
    vx, vy, vz = map(finite, vector)
    # Unit quaternion vector rotation, without tf2_py/C++ ABI dependencies.
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx]


def inverse_transform(translation, quaternion):
    x, y, z, w = map(finite, quaternion)
    norm2 = x*x+y*y+z*z+w*w
    if norm2 < 1e-18: raise ValueError('Invalid quaternion')
    inverse = [-x/norm2, -y/norm2, -z/norm2, w/norm2]
    return rotate([-finite(v) for v in translation], inverse), inverse


def front_point_map(position, raw_heading, translation, quaternion, offset=3.):
    """Raw MORAI Ego only: apply -1.347deg once, then existing map<-morai TF."""
    yaw = math.radians(finite(raw_heading)+HEADING_CORRECTION_DEGREES)
    offset = finite(offset)
    local = [finite(position[0])+offset*math.cos(yaw),
             finite(position[1])+offset*math.sin(yaw), finite(position[2])]
    rotated = rotate(local, quaternion)
    return [rotated[index]+finite(translation[index]) for index in range(3)]


def native_bracket(samples, native_id, crossing_monotonic, max_age_sec):
    """Conservative request/response brackets, never nearest-state extrapolation."""
    rows = [row for row in samples if row.get('native_id') == native_id and row.get('valid')]
    before = [row for row in rows if row['response_monotonic'] <= crossing_monotonic]
    after = [row for row in rows if row['request_monotonic'] >= crossing_monotonic]
    left = max(before, key=lambda row: row['response_monotonic'], default=None)
    right = min(after, key=lambda row: row['request_monotonic'], default=None)
    result = {'verified': False, 'native_id': native_id, 'max_age_sec': max_age_sec,
              'before_record_id': left.get('record_id') if left else None,
              'after_record_id': right.get('record_id') if right else None,
              'before_color': left.get('color') if left else None,
              'after_color': right.get('color') if right else None}
    if left is None or right is None:
        result['reason'] = 'missing_before_or_after_native_sample'
        return result
    before_age = crossing_monotonic-left['request_monotonic']
    after_age = right['response_monotonic']-crossing_monotonic
    result.update(before_age_bound_sec=before_age, after_age_bound_sec=after_age)
    if before_age >= max_age_sec or after_age >= max_age_sec:
        result['reason'] = 'native_GT_age_at_or_over_bound'
    elif left['color'] != right['color']:
        result['reason'] = 'native_state_transition_crosses_bracket'
    else:
        # Retain intervening failure attempts; do not bridge one silently.
        failures = [row['record_id'] for row in samples if row.get('native_id') == native_id
                    and not row.get('valid') and row['response_monotonic'] >= left['response_monotonic']
                    and row['request_monotonic'] <= right['request_monotonic']]
        if failures:
            result.update(reason='intervening_native_read_failure', failed_record_ids=failures)
        else:
            result.update(verified=True, reason='same_exact_native_state_before_after', color=left['color'])
    return result


def evaluate_crossing(event, samples, group):
    results = {}
    for bound in AGE_THRESHOLDS_SEC:
        heads = [native_bracket(samples, native_id, event['estimated_source_monotonic'], bound)
                 for native_id in group['native_ids']]
        verified = event['pose_time_verified'] and all(head['verified'] for head in heads)
        colors = {head.get('color') for head in heads if head['verified']}
        if len(colors) != 1: verified = False
        color = next(iter(colors)) if verified else None
        required_bit = 32 if group['group_id'] == 201326 else 16
        results[str(bound)] = {'verified': bool(verified), 'native_heads': heads,
                               'color': color, 'route_permission_proxy': bool(color & required_bit) if verified else None,
                               'required_native_bit': required_bit,
                               'reason': ('verified_proxy_not_official_penalty' if verified else
                                          'pose_or_native_bracket_unverified_or_C_heads_disagree')}
    return results


def stamp_ns(value):
    if value is None: return None
    for sec, nsec in (('secs', 'nsecs'), ('sec', 'nanosec')):
        if hasattr(value, sec) and hasattr(value, nsec):
            return int(getattr(value, sec))*1000000000+int(getattr(value, nsec))
    return value.to_nsec() if hasattr(value, 'to_nsec') else None


def primitive(value):
    if value is None or isinstance(value, (str, bool, int)): return value
    if isinstance(value, float): return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)): return [primitive(item) for item in value]
    if isinstance(value, dict): return {str(key): primitive(item) for key, item in value.items()}
    if hasattr(value, '__slots__'): return {name: primitive(getattr(value, name)) for name in value.__slots__}
    return str(value)


class JsonlWriter:
    def __init__(self, path):
        self.stream, self.lock, self.count = path.open('x'), threading.Lock(), 0

    def write(self, row):
        with self.lock:
            self.count += 1
            row = {**primitive(row), 'record_id': self.count}
            self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
            self.stream.flush()
            return row['record_id']

    def close(self):
        with self.lock: self.stream.close()


class CrossingJpegRing:
    """Bounded original-byte memory; persist at most two frames per crossing.

    JPEG validity belongs to compressed_jpeg() at the subscriber boundary.
    No decoding, re-encoding, lamp inference or native-GT replacement occurs.
    A source-time watermark past crossing+.1s closes the candidate window.
    """
    def __init__(self, output, emit=None):
        self.output, self.emit = Path(output), emit or (lambda row: None)
        self.frames, self.pending = deque(maxlen=20), []

    def add(self, payload, source_ns, receipt_ros_ns, receipt_monotonic, **details):
        frame = dict(payload=bytes(payload), source_stamp_ns=source_ns, receipt_ros_ns=receipt_ros_ns,
                     receipt_monotonic=receipt_monotonic, **details)
        self.frames.append(frame)
        while self.frames and receipt_monotonic-self.frames[0]['receipt_monotonic'] > .5:
            self.frames.popleft()
        for item in list(self.pending):
            self._consider(item, frame)
            if type(source_ns) is int and source_ns > item['crossing_source_stamp_ns']+100000000:
                self._finish(item, 'source_window_closed')

    def begin(self, event):
        result = dict(status='pending', camera_topic='/sensor/camera/front/compressed',
                      before=None, after=None, max_source_delta_sec=.1,
                      timing_bracket_verified=False, independent_pixel_review_required=True,
                      native_GT_replaced=False)
        event['crossing_jpeg_bracket'] = result
        source = event.get('estimated_source_stamp_ns')
        if type(source) is not int or source <= 0:
            result.update(status='missing', reason='crossing_source_stamp_unverified')
            return
        item = dict(event=event, result=result, crossing_source_stamp_ns=source, before=None, after=None)
        self.pending.append(item)
        for frame in self.frames: self._consider(item, frame)
        available = [frame['source_stamp_ns'] for frame in self.frames if type(frame['source_stamp_ns']) is int]
        if available and max(available) > source+100000000:
            self._finish(item, 'source_window_already_closed')

    def _consider(self, item, frame):
        source = frame['source_stamp_ns']
        if type(source) is not int or source <= 0: return
        delta = source-item['crossing_source_stamp_ns']
        # One exactly coincident image is not two independent sides of a bracket.
        if delta == 0 or abs(delta) > 100000000: return
        side = 'before' if delta < 0 else 'after'
        previous = item[side]
        if previous is None or abs(delta) < abs(previous['source_stamp_ns']-item['crossing_source_stamp_ns']):
            item[side] = frame

    def _finish(self, item, reason):
        if item not in self.pending: return
        result, event = item['result'], item['event']
        for side in ('before', 'after'):
            frame = item[side]
            if frame is None:
                result[side] = dict(status='missing', reason='no_'+side+'_JPEG_within_0.1s')
                continue
            self.output.mkdir(parents=True, exist_ok=True)
            path = self.output/('crossing_%08d_%s.jpg' % (event['record_id'], side))
            with path.open('xb') as stream: stream.write(frame['payload'])
            source_age = (frame['receipt_ros_ns']-frame['source_stamp_ns'])/1e9
            result[side] = {key: value for key, value in frame.items() if key != 'payload'}
            result[side].update(status='saved_original_JPEG', path=str(path),
                sha256=hashlib.sha256(frame['payload']).hexdigest(), bytes=len(frame['payload']),
                source_delta_to_crossing_sec=(frame['source_stamp_ns']-item['crossing_source_stamp_ns'])/1e9,
                receipt_delta_to_crossing_estimated_monotonic_sec=frame['receipt_monotonic']-event['estimated_source_monotonic'],
                source_age_at_receipt_sec=source_age, source_clock_verified=0 <= source_age < .5)
        complete = item['before'] is not None and item['after'] is not None
        result.update(status='complete_original_pair' if complete else 'missing_side', selection_closed_reason=reason,
            timing_bracket_verified=bool(complete and event['pose_time_verified'] and
                result['before']['source_clock_verified'] and result['after']['source_clock_verified']))
        self.emit(dict(event='crossing_JPEG_bracket_material', crossing_event_record_id=event['record_id'],
                       crossing_jpeg_bracket=result))
        self.pending.remove(item)

    def close(self):
        for item in list(self.pending): self._finish(item, 'observer_closed')


def run(args):
    # Lazy runtime dependencies keep geometry/bracket tests ROS/grpc-free.
    import rospy
    from roslib.message import get_message_class
    from capture_traffic_light_phase_sequence import ModernNativeReader, native_readback_issue

    groups, geometry = load_geometry(args.geometry)
    if not 0 < args.duration_sec <= 1800 or not 2 <= args.native_poll_hz <= 5:
        raise ValueError('duration 0..1800 seconds and native polling 2..5Hz required')
    if not 0 < args.native_timeout_sec <= .5 or not 0 <= args.front_axle_offset_m <= 6:
        raise ValueError('Native timeout 0..0.5s and front offset0..6m required')
    output = args.output.expanduser().resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists(): raise FileExistsError('Fresh artifacts directory required')
    output.mkdir(parents=True)
    metadata = {'development_only': True, 'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'geometry': geometry, 'args': vars(args), 'GT_pose': 'raw Ego, heading-1.347 once, existing map<-morai_map TF, front offset',
                'front_axle_offset_m': args.front_axle_offset_m, 'GT_age_bounds_sec': AGE_THRESHOLDS_SEC,
                'limitations': ['Development proxy, not official stop-line or penalty certification.',
                    'Offset and supplied stop-line coordinates need independent physical validation.',
                    'Native request-response brackets do not prove render transitions or exclude an unseen transient.',
                    'No TF reconstruction, no control publisher, no signal Set, no stack startup.',
                    'Ego/localization are separate; raw Ego is evaluation-only, not an operational fusion input.',
                    'No crossing is not proof of a complete drive: inspect message/error counts and geometry.']}
    metadata['crossing_jpegs'] = dict(enabled=args.crossing_jpegs, memory_ring_max_frames=20,
        memory_ring_max_receipt_age_sec=.5, save_max_source_delta_sec=.1,
        image_encoding='unchanged original compressed JPEG bytes', full_camera_dump=False,
        pixel_lamp_GT='independent manual review required; native unknown stays unknown')
    metadata['args'] = {key: str(value) if isinstance(value, Path) else value for key, value in metadata['args'].items()}
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    writer, stop, lock = JsonlWriter(output/'observations.jsonl'), threading.Event(), threading.RLock()
    image_ring = CrossingJpegRing(output/'crossing_jpegs', writer.write) if args.crossing_jpegs else None
    native_samples, crossings, counts, state = [], [], Counter(), {'tf': None, 'previous_ego': None}
    rospy.init_node('traffic_light_compliance_observer', anonymous=True, disable_signals=True)
    deadline = time.monotonic()+args.duration_sec
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, lambda *_: stop.set())

    def tf_observation(message, topic, mono, ros_ns):
        for transform in message.transforms:
            parent, child = transform.header.frame_id.strip('/'), transform.child_frame_id.strip('/')
            if (parent, child) not in (('map', 'morai_map'), ('morai_map', 'map')): continue
            position, orientation = transform.transform.translation, transform.transform.rotation
            translation, quaternion = [position.x, position.y, position.z], [orientation.x, orientation.y, orientation.z, orientation.w]
            if parent != 'map': translation, quaternion = inverse_transform(translation, quaternion)
            with lock:
                state['tf'] = {'translation': translation, 'quaternion_xyzw': quaternion,
                               'source_stamp_ns': stamp_ns(transform.header.stamp),
                               'is_static': topic == '/tf_static', 'receipt_monotonic': mono,
                               'receipt_ros_ns': ros_ns, 'original_parent': parent, 'original_child': child}

    def ego_observation(message, mono, ros_ns):
        source_ns = stamp_ns(getattr(getattr(message, 'header', None), 'stamp', None))
        age = (ros_ns-source_ns)/1e9 if source_ns and source_ns > 0 else None
        with lock:
            transform = state['tf']
            if transform is None:
                writer.write({'event': 'ego_GT_unverified', 'reason': 'direct_map_morai_map_TF_missing',
                              'receipt_monotonic': mono, 'source_stamp_ns': source_ns})
                state['previous_ego'] = None
                return
            tf_age = (ros_ns-transform['source_stamp_ns'])/1e9 if transform['source_stamp_ns'] else None
            if not transform['is_static'] and (tf_age is None or not 0 <= tf_age < .5):
                writer.write({'event': 'ego_GT_unverified', 'reason': 'dynamic_TF_time_unverified', 'TF': transform})
                state['previous_ego'] = None
                return
            front = front_point_map([message.position.x, message.position.y, message.position.z],
                                     message.heading, transform['translation'], transform['quaternion_xyzw'],
                                     args.front_axle_offset_m)
            current = {'front_point_map_xyz_m': front, 'raw_heading_degrees': message.heading,
                       'corrected_native_heading_degrees': message.heading+HEADING_CORRECTION_DEGREES,
                       'source_stamp_ns': source_ns, 'source_age_sec': age, 'receipt_ros_ns': ros_ns,
                       'receipt_monotonic': mono, 'estimated_source_monotonic': mono-age if age is not None else mono,
                       'pose_time_verified': age is not None and 0 <= age < .25, 'TF': dict(transform)}
            writer.write({'event': 'ego_front_GT_proxy', **current})
            previous, state['previous_ego'] = state['previous_ego'], current
            if previous is None: return
            dt = mono-previous['receipt_monotonic']
            source_dt = current['estimated_source_monotonic']-previous['estimated_source_monotonic']
            if (not 0 < dt < .5 or not 0 < source_dt < .5
                    or math.dist(front, previous['front_point_map_xyz_m']) > 20):
                writer.write({'event': 'crossing_interval_unverified', 'reason': 'ego_gap_or_position_jump', 'previous': previous, 'current': current})
                state['previous_ego'] = None
                return
            for group in groups:
                points = group['stop_line_xy']
                for index, (a, b) in enumerate(zip(points, points[1:])):
                    crossing = segment_crossing(previous['front_point_map_xyz_m'][:2], front[:2], a, b,
                                                group.get('crossing_direction', 'both'))
                    if crossing is None: continue
                    fraction = crossing['motion_fraction']
                    event = {'event': 'front_point_segment_crossing_pending_native_bracket', 'group_id': group['group_id'],
                             'segment_index': index, **crossing, 'previous_ego': previous, 'current_ego': current,
                             'pose_time_verified': previous['pose_time_verified'] and current['pose_time_verified'],
                             'estimated_source_monotonic': previous['estimated_source_monotonic']+fraction*(
                                 current['estimated_source_monotonic']-previous['estimated_source_monotonic'])}
                    if previous['source_stamp_ns'] and current['source_stamp_ns']:
                        event['estimated_source_stamp_ns'] = previous['source_stamp_ns']+round(fraction*(
                            current['source_stamp_ns']-previous['source_stamp_ns']))
                    event['record_id'] = writer.write(event)
                    crossings.append(event)
                    if image_ring is not None: image_ring.begin(event)

    def callback(topic):
        def receive(raw):
            mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
            with lock:
                if stop.is_set(): return
                try:
                    msgtype = raw._connection_header.get('type')
                    cls = get_message_class(msgtype)
                    if cls is None: raise ValueError('ROS message type unavailable: '+str(msgtype))
                    message = cls().deserialize(raw._buff)
                    counts[topic] += 1
                    if topic == '/sensor/camera/front/compressed' and image_ring is not None:
                        from sample_traffic_light_bag_frames import compressed_jpeg
                        payload, width, height = compressed_jpeg(message)
                        image_ring.add(payload, stamp_ns(message.header.stamp), ros_ns, mono,
                                       frame_id=message.header.frame_id, format=message.format,
                                       width=width, height=height)
                        # Store timing/count metadata, never a complete JPEG stream.
                        writer.write(dict(event='camera_JPEG_ring_header', source_stamp_ns=stamp_ns(message.header.stamp),
                            receipt_ros_ns=ros_ns, receipt_monotonic=mono, bytes=len(payload), width=width, height=height))
                        return
                    payload = primitive(message)
                    if msgtype == 'std_msgs/String':
                        try: payload['parsed_json'] = json.loads(message.data)
                        except (TypeError, ValueError): pass
                    writer.write({'event': 'ROS_observation', 'topic': topic, 'message_type': msgtype,
                                  'receipt_ros_ns': ros_ns, 'receipt_monotonic': mono, 'message': payload})
                    if topic in ('/tf', '/tf_static'): tf_observation(message, topic, mono, ros_ns)
                    elif topic == '/Ego_topic': ego_observation(message, mono, ros_ns)
                except Exception as error:
                    counts['error:'+topic] += 1
                    writer.write({'event': 'ROS_observation_error', 'topic': topic, 'receipt_ros_ns': ros_ns,
                                  'receipt_monotonic': mono, 'error': type(error).__name__+':'+str(error)})
        return receive

    topics = ['/Ego_topic', '/localization/kinematic_state', '/ctrl_cmd', '/tf', '/tf_static',
              args.signal_topic, args.route_group_topic, '/planning/mission_planning/route']
    topics += [args.node_namespace.rstrip('/')+'/'+name for name in ('debug_selection', 'debug_classifier',
               'debug_policy_reason', 'debug_observed_label', 'debug_raw_label', 'debug_runtime')]
    if args.crossing_jpegs: topics.append('/sensor/camera/front/compressed')
    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, callback(topic), queue_size=50, buff_size=2**22)
                     for topic in topics]
    reader = None
    try:
        reader = ModernNativeReader(endpoint=args.native_endpoint, timeout_sec=args.native_timeout_sec, rpc_retries=0)
        poll = 0
        while not stop.is_set() and not rospy.is_shutdown() and time.monotonic() < deadline:
            scheduled = time.monotonic()
            for native_id in [value for group in groups for value in group['native_ids']]:
                if stop.is_set() or time.monotonic() >= deadline: break
                row = {'event': 'native_read_only_sample', 'native_id': native_id, 'poll_index': poll,
                       'request_monotonic': time.monotonic(), 'request_ros_ns': rospy.Time.now().to_nsec(), 'valid': False}
                try:
                    readback = reader(native_id, deadline=deadline)
                    issue = native_readback_issue(readback, native_id)
                    row.update(readback=readback, valid=issue is None, validation_issue=issue)
                    if issue is None: row['color'] = readback['observations'][0]['tl_color']
                except Exception as error:
                    row.update(error=type(error).__name__+':'+str(error), rpc_attempts=reader.last_rpc_attempts)
                row.update(response_monotonic=time.monotonic(), response_ros_ns=rospy.Time.now().to_nsec())
                row['record_id'] = writer.write(row)
                native_samples.append(row)
            poll += 1
            stop.wait(max(0., min(deadline-time.monotonic(), 1/args.native_poll_hz-(time.monotonic()-scheduled))))
    finally:
        stop.set()
        for subscription in subscriptions: subscription.unregister()
        if reader is not None: reader.close()
        # Allow an already-running callback to leave the small state section.
        with lock:
            if image_ring is not None: image_ring.close()
            results = []
            for event in crossings:
                group = next(group for group in groups if group['group_id'] == event['group_id'])
                result = {**event, 'native_bracket_evaluation': evaluate_crossing(event, native_samples, group)}
                results.append(result)
                writer.write({**result, 'event': 'crossing_proxy_evaluation',
                              'crossing_event_record_id': event['record_id']})
            (output/'crossings.json').write_text(json.dumps({'development_only': True, 'crossings': results,
                'ROS_message_counts': dict(counts), 'native_samples': len(native_samples),
                'native_valid_samples': sum(row['valid'] for row in native_samples),
                'no_crossing_is_not_success': True}, indent=2, allow_nan=False)+'\n')
        writer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--front-axle-offset-m', type=float, default=3.)
    parser.add_argument('--duration-sec', type=float, default=180.)
    parser.add_argument('--native-poll-hz', type=float, default=5.)
    parser.add_argument('--native-timeout-sec', type=float, default=.15)
    parser.add_argument('--native-endpoint', default='127.0.0.1:7789')
    parser.add_argument('--signal-topic', default='/perception/traffic_light_recognition/traffic_signals')
    parser.add_argument('--route-group-topic', default='/perception/traffic_light_recognition/route_group_id')
    parser.add_argument('--node-namespace', default='/traffic_light_node')
    parser.add_argument('--crossing-jpegs', action='store_true',
                        help='Candidate: retain only original JPEG pairs within0.1s around finite-segment crossings')
    run(parser.parse_args())


if __name__ == '__main__': main()
