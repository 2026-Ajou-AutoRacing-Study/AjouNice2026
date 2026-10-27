#!/usr/bin/env python3
"""Development-only signal stress sequence; never commands a vehicle/stack.

Only the selected A/B/C/D native IDs receive official signal overrides.
Persistent gRPC is the default; the historical UDP impulse transport is an
explicit reproduction option, not a qualified stable phase program.
ROS output is solely the explicit evaluation fault-control String topic. The
root-owned proxy may drop/change the ID of real v5 output; no GT colors enter
that topic. A controller/program error exits nonzero for the owning watchdog.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

from observe_traffic_light_compliance import (ROOT, GROUP_IDS, JsonlWriter, finite,
                                             load_geometry, primitive, rotate, segment_crossing, stamp_ns)

FAULT_TOPIC = '/evaluation/traffic_light_policy/fault'
FRESH_SEC = 1.
START_HOLD_SEC = 1.
APPLY_TIMEOUT_SEC = 2.
MULTI_HEAD_FRAME_WAIT_SEC = .1
TERMINAL_CROSSING_TAIL_SEC = 1.


class TrialError(RuntimeError):
    pass


def parse_group(value):
    aliases = {'A': 201351, 'B': 201365, 'C': 201290, 'D': 201326, 'D-2': 201326}
    try: group = aliases[value.upper()] if value.upper() in aliases else int(value)
    except (ValueError, AttributeError): raise ValueError('Use A/B/C/D or its explicit mission group ID')
    if group not in GROUP_IDS: raise ValueError('Only the four explicit mission groups are supported; old D-1 is excluded')
    return group


def controlled_go_state(group_id):
    if type(group_id) is not int or group_id not in GROUP_IDS:
        raise ValueError('Only the four explicit mission groups are supported')
    return {201351:16, 201365:16, 201290:48, 201326:33}[group_id]


def yellow_durations(values=None):
    values = (1., 3., 5.) if values is None else values
    if not isinstance(values, (list, tuple)) or not 1 <= len(values) <= 12:
        raise ValueError('Use a bounded list of 1..12 yellow durations')
    result = tuple(finite(value) for value in values)
    if any(not 0 < value <= 10 for value in result):
        raise ValueError('Each yellow duration must be finite, positive and <=10 seconds')
    return result


def parse_yellow_durations(value):
    try:
        return yellow_durations(tuple(float(item.strip()) for item in value.split(',')))
    except (ValueError, TypeError, AttributeError) as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def initial_arming_phase(group_id, experiment):
    controlled_go_state(group_id)  # Validate an exact mission group before SET.
    if experiment == 'first-observation-yellow':
        return (5 if group_id == 201326 else 4), 'arming_first_observation_yellow'
    return 1, 'arming_red'


def experiment_stages(group_id, experiment, yellow_durations_sec=None):
    go = controlled_go_state(group_id)
    durations = yellow_durations((3.,) if experiment == 'first-observation-yellow'
                                and yellow_durations_sec is None else yellow_durations_sec)
    yellow = 5 if group_id == 201326 else 4
    terminal = dict(stage='terminal_controlled_go', action='controlled_go_until_owner_teardown',
                    color=go, seconds=None, fault='none')
    if experiment == 'moving-native-yellow':
        if len(durations) != 1 or durations[0] not in (1., 2., 3.):
            raise ValueError('Moving native trial requires exactly one yellow duration: 1, 2 or 3 seconds')
        terminal['action'] = 'controlled_go_until_crossing_then_owner_teardown'
        return [dict(stage='moving_yellow', color=yellow, seconds=durations[0], fault='none'),
                dict(stage='red_after_moving_yellow', color=1, seconds=3., fault='none'), terminal]
    if experiment == 'first-observation-yellow':
        if len(durations) != 1 or durations[0] not in (1., 2., 3.):
            raise ValueError('First-observation yellow requires one explicit 1, 2 or 3 second post-stop phase')
        terminal['action'] = 'controlled_go_until_crossing_then_owner_teardown'
        return [dict(stage='first_observation_yellow_after_stop', color=yellow,
                     seconds=durations[0], fault='none'),
                dict(stage='red_after_first_observation_yellow', color=1, seconds=2., fault='none'), terminal]
    if experiment == 'stopped-yellow':
        return [dict(stage='yellow_%ss' % format(duration, 'g'), color=yellow, seconds=duration, fault='none')
                if index % 2 == 0 else dict(stage='red_after_%ss' % format(duration, 'g'), color=1, seconds=2., fault='none')
                for duration in durations for index in (0, 1)] + [
                    terminal]
    if experiment == 'missing-signal':
        return [dict(stage='red_drop', color=1, seconds=3., fault='drop'),
                dict(stage='red_wrong_id', color=1, seconds=3., fault='wrong_id'),
                dict(stage='red_restored', color=1, seconds=2., fault='none'),
                terminal]
    raise ValueError('Unsupported experiment')


def moving_front_geometry(odom, group, offset=3.):
    """Finite stop-segment intersection along current heading, signed metres."""
    if odom.get('frame', '').strip('/') != 'map':
        return dict(ready=False, reason='odom_not_map')
    forward = rotate([1., 0., 0.], odom['quaternion_xyzw'])
    norm = math.hypot(*forward[:2])
    if norm < 1e-6:
        return dict(ready=False, reason='invalid_heading')
    f = [x/norm for x in forward[:2]]
    point = [finite(odom['position_xyz'][i])+offset*f[i] for i in (0, 1)]
    for a, b in zip(group['stop_line_xy'], group['stop_line_xy'][1:]):
        vx, vy = b[0]-a[0], b[1]-a[1]
        denominator = vx*f[1]-vy*f[0]
        if abs(denominator) < 1e-8:
            continue
        distance = -(vx*(point[1]-a[1])-vy*(point[0]-a[0]))/denominator
        hit = [point[i]+distance*f[i] for i in (0, 1)]
        fraction = ((hit[0]-a[0])*vx+(hit[1]-a[1])*vy)/(vx*vx+vy*vy)
        if 0 <= fraction <= 1 and -10 <= distance <= 100:
            return dict(ready=True, front_distance_m=distance, finite_segment_fraction=fraction,
                        front_point_map_xy_m=point)
    return dict(ready=False, reason='heading_does_not_intersect_finite_stop_segment')


def moving_trigger_distance(value):
    value = finite(value)
    if not 1. <= value <= 60.:
        raise ValueError('Moving trigger front distance must be within 1..60m')
    return value


class TerminalCrossingRelease:
    """Stop overriding a passed head, without ending owner/fault heartbeats.

    Uses actual map-odometry front-point motion across the explicit finite line,
    not a timer since departure or a requested lamp state. The independent
    observer still owns native-state qualification of the crossing.
    """
    def __init__(self, group, offset=3., tail_sec=TERMINAL_CROSSING_TAIL_SEC):
        self.group, self.offset, self.tail_sec = group, offset, finite(tail_sec)
        if not 0 <= self.tail_sec <= 5.:
            raise ValueError('Terminal crossing tail must be within 0..5 seconds')
        self.previous, self.crossing = None, None

    def update(self, odom, now):
        if self.crossing is not None:
            return None
        source = odom.get('source_stamp_ns')
        if odom.get('frame', '').strip('/') != 'map' or type(source) is not int or source <= 0:
            self.previous = None
            return None
        forward = rotate([1., 0., 0.], odom['quaternion_xyzw'])
        norm = math.hypot(*forward[:2])
        if norm < 1e-6:
            self.previous = None
            return None
        point = [finite(odom['position_xyz'][i]) + self.offset * forward[i] / norm for i in (0, 1)]
        current = dict(point=point, source=source, monotonic=finite(now))
        previous, self.previous = self.previous, current
        if previous is None:
            return None
        gap = (source - previous['source']) / 1e9
        if not 0 < gap <= 1. or math.dist(previous['point'], point) > 10.:
            return None
        for a, b in zip(self.group['stop_line_xy'], self.group['stop_line_xy'][1:]):
            crossing = segment_crossing(previous['point'], point, a, b,
                                        self.group.get('crossing_direction', 'both'))
            if crossing is not None:
                self.crossing = dict(crossing, observed_monotonic=now,
                                     before_source_stamp_ns=previous['source'], after_source_stamp_ns=source)
                return self.crossing
        return None

    def ready(self, now):
        return self.crossing is not None and finite(now) - self.crossing['observed_monotonic'] >= self.tail_sec


def moving_trigger_gate(latest, group, now, ros_ns, trigger_distance_m, offset=3.):
    fresh, reason = fresh_inputs(latest, now, ros_ns)
    if not fresh:
        raise TrialError('Moving trial input invalid: '+reason)
    geometry = moving_front_geometry(latest['odom'], group, offset)
    speed = finite(latest['odom']['velocity_xyz'][0])
    signal_row = latest.get('signal', {})
    signal_age = now-signal_row.get('receipt_monotonic', -1e30)
    source_age = (ros_ns-signal_row.get('source_stamp_ns', 0))/1e9
    authorized = bool(signal_row.get('authorized')) and 0 <= signal_age <= .5 and 0 <= source_age <= .5
    return dict(ready=geometry['ready'] and .2 < geometry['front_distance_m'] <= trigger_distance_m
                      and speed >= 1. and latest['gear']['value'] == 4 and authorized,
                geometry=geometry, speed_mps=speed, route_authorized=authorized,
                signal_source_stamp_ns=signal_row.get('source_stamp_ns'))


def fresh_inputs(latest, now, ros_ns):
    for name in ('odom', 'gear', 'ctrl'):
        row = latest.get(name)
        if row is None: return False, name+'_missing'
        age = now-row['receipt_monotonic']
        if not 0 <= age < FRESH_SEC: return False, name+'_receipt_stale_or_clock_reversed'
        source = row.get('source_stamp_ns')
        # Some MORAI CtrlCmd/gear variants are unstamped; receipt freshness is
        # the only claim for those. Odometry must have its own valid source time.
        if name == 'odom' and (type(source) is not int or source <= 0):
            return False, 'odom_source_stamp_missing'
        if source is not None and source > 0 and not 0 <= (ros_ns-source)/1e9 < FRESH_SEC:
            return False, name+'_source_stale_or_future'
    try:
        odom = latest['odom']
        for key, length in (('position_xyz', 3), ('velocity_xyz', 3), ('quaternion_xyzw', 4)):
            if len(odom[key]) != length: return False, 'odom_invalid_geometry'
            for value in odom[key]: finite(value)
        if sum(value*value for value in odom['quaternion_xyzw']) < 1e-12:
            return False, 'odom_invalid_quaternion'
    except (TypeError, KeyError, ValueError):
        return False, 'odom_invalid_geometry'
    return True, 'fresh_inputs'


def wait_for_preparation_inputs(snapshot, monotonic, ros_now_ns, stopped, wait,
                                deadline, emit, publisher_error=lambda: None):
    """Bounded raw-input preparation; P is allowed and no native calls exist here.

    A validator may still be loading its scene before any message arrives. Do
    not Set/reset/read a native signal during that input-less preparation gap.
    The later start gate independently requires D4 and stopped-before-line.
    """
    while True:
        if stopped(): raise TrialError('Interrupted while waiting for initial inputs')
        now = monotonic()
        if now >= deadline: raise TrialError('Initial input preparation global deadline exceeded')
        if publisher_error() is not None: raise TrialError('Fault-control publisher failed during preparation')
        fresh, reason = fresh_inputs(snapshot(), now, ros_now_ns())
        emit('preparation_input_observation', ready=fresh, reason=reason)
        if fresh: return
        wait(min(.1, max(0., deadline-monotonic())))


def front_stop_geometry(odom, group, offset=3.):
    """Localization is already corrected: do NOT apply raw Ego -1.347 again."""
    if odom.get('frame', '').strip('/') != 'map': return {'ready': False, 'reason': 'odom_not_map'}
    position = list(map(finite, odom['position_xyz']))
    forward = rotate([1., 0., 0.], odom['quaternion_xyzw'])
    norm_xy = math.hypot(*forward[:2])
    if norm_xy < 1e-6: return {'ready': False, 'reason': 'invalid_planar_heading'}
    forward = [value/norm_xy for value in forward[:2]]
    front = [position[index]+finite(offset)*forward[index] for index in (0, 1)]
    matches = []
    for index, (a, b) in enumerate(zip(group['stop_line_xy'], group['stop_line_xy'][1:])):
        vx, vy = finite(b[0])-finite(a[0]), finite(b[1])-finite(a[1])
        length = math.hypot(vx, vy)
        if length < 1e-6: raise ValueError('Degenerate stop-line segment')
        side = vx*(front[1]-a[1])-vy*(front[0]-a[0])
        derivative = vx*forward[1]-vy*forward[0]
        if abs(derivative) < 1e-8: continue
        ahead = -side/derivative
        point = [front[axis]+ahead*forward[axis] for axis in (0, 1)]
        along = ((point[0]-a[0])*vx+(point[1]-a[1])*vy)/(length*length)
        distance = abs(side)/length
        if 0 <= along <= 1 and .2 <= distance <= 2. and .2 <= ahead <= 2.:
            matches.append(dict(segment_index=index, front_point_map_xy_m=front,
                                intersection_xy_m=point, finite_segment_fraction=along,
                                perpendicular_distance_m=distance, forward_distance_m=ahead))
    return {'ready': bool(matches), 'reason': 'in_front_of_finite_stop_segment' if matches else 'outside_forward_stop_gate',
            'matches': matches}


def start_gate(latest, group, now, ros_ns, offset=3.):
    fresh, reason = fresh_inputs(latest, now, ros_ns)
    if not fresh: return {'ready': False, 'reason': reason}
    if latest['gear']['value'] != 4: return {'ready': False, 'reason': 'gear_not_D4'}
    velocity = list(map(finite, latest['odom']['velocity_xyz']))
    speed = math.sqrt(sum(value*value for value in velocity))
    if speed >= .1: return {'ready': False, 'reason': 'not_stationary', 'speed_mps': speed}
    return {**front_stop_geometry(latest['odom'], group, offset), 'speed_mps': speed}


class StartHold:
    def __init__(self): self.since = None

    def update(self, ready, now):
        if not ready or self.since is not None and now < self.since:
            self.since = None
        elif self.since is None:
            self.since = now
        return self.since is not None and now-self.since >= START_HOLD_SEC


class PhaseClock:
    def __init__(self, stage, now):
        self.stage, self.requested_at, self.confirmed_at = stage, now, None

    def update(self, now, actual_matches):
        if now < self.requested_at or self.confirmed_at is not None and now < self.confirmed_at:
            raise TrialError('Phase monotonic clock regressed')
        if self.confirmed_at is None:
            if actual_matches: self.confirmed_at = now
            elif now-self.requested_at >= APPLY_TIMEOUT_SEC:
                raise TrialError('Requested native phase was not confirmed within bounded apply wait')
        elif not actual_matches:
            raise TrialError('Native phase changed after confirmation despite repeated override')
        return self.confirmed_at is not None and now-self.confirmed_at >= self.stage['seconds']


def send_positive_signal(sender, native_id, state):
    """Shared last-wire guard: this trial never sends negative/reset/bool states."""
    if type(state) is not int or state not in {1, 4, 5, 16, 33, 48}:
        raise ValueError('Explicit approved positive signal state required before UDP send')
    return sender(native_id, state)


def coordinated_native_cycle(native_ids, state, send, read, frame_wait=None):
    """SET every selected head before GET of any head; this is not atomic GT.

    Geometry validation independently pins exact mission IDs. Per-head send/read
    observations must remain separate; no synthetic common color is produced.
    Multi-head commands need simulator frame consumption between datagrams:
    back-to-back C016/C018 SET can leave only the last head overridden.
    Single-head cadence is unchanged. Wait after each multi-head SET, including
    the last, then independently GET both heads. No failed read is accepted.
    """
    if (not isinstance(native_ids, (list, tuple)) or not native_ids or
            any(not isinstance(item, str) or not item.strip() for item in native_ids) or
            len(set(native_ids)) != len(native_ids)):
        raise ValueError('Unique explicit native IDs are required')
    wait = time.sleep if frame_wait is None else frame_wait
    for native_id in native_ids:
        send_positive_signal(send, native_id, state)
        if len(native_ids) > 1:
            wait(MULTI_HEAD_FRAME_WAIT_SEC)
    return {native_id: read(native_id) for native_id in native_ids}


def native_transport(value):
    if value not in ('persistent', 'udp'):
        raise ValueError('Native transport must be persistent or explicit legacy udp')
    return value


def make_native_sender(transport):
    """Lazy imports avoid the short-cycle helper's shared-gate import cycle."""
    if native_transport(transport) == 'persistent':
        from control_traffic_light_short_cycles import PersistentNativeSender
        return PersistentNativeSender()
    from capture_traffic_light_phase_sequence import OfficialSender
    return OfficialSender()


def apply_native_cycle(native_ids, color, send, read, transport, frame_wait=None):
    if native_transport(transport) == 'persistent':
        from control_traffic_light_short_cycles import paced_native_cycle
        return paced_native_cycle(native_ids, color, send, read,
                                  frame_wait=time.sleep if frame_wait is None else frame_wait)
    return coordinated_native_cycle(native_ids, color, send, read, frame_wait=frame_wait)


class TerminalPositiveHold:
    """Bounded initial positive-GO confirmation, then keep it until owner SIGTERM.

    No fixed GO duration and no natural-program restoration claim. Caller owns
    repeated SET/GET, fresh-input/deadline checks and finally sends no signals.
    """
    def __init__(self, color, started_at):
        if type(color) is not int or color not in {16, 33, 48}:
            raise ValueError('Terminal controlled GO must be A/B16, C48 or D33')
        self.color, self.requested_at, self.confirmed_at = color, started_at, None

    def update(self, now, actual_matches):
        if now < self.requested_at or self.confirmed_at is not None and now < self.confirmed_at:
            raise TrialError('Terminal positive-GO monotonic clock regressed')
        if self.confirmed_at is None:
            if actual_matches:
                self.confirmed_at = now
                return True
            if now-self.requested_at >= APPLY_TIMEOUT_SEC:
                raise TrialError('Terminal controlled GO was not confirmed within bounded apply wait')
        elif not actual_matches:
            raise TrialError('Terminal controlled GO changed after confirmation')
        return False

    def finish_on_signal(self, signum):
        if signum != signal.SIGTERM or self.confirmed_at is None:
            raise TrialError('Terminal controlled GO not confirmed or not normal owner SIGTERM')
        return True


def fault_payload(mode, group_id, experiment, stage):
    if mode not in ('none', 'drop', 'wrong_id'): raise ValueError('Unsupported fault mode')
    # No native state, color, lamp array, GO, or synthetic planner message.
    return dict(mode=mode, group_id=group_id, experiment=experiment, stage=stage)


class FaultRepeater:
    """0.1s output cadence does not wait for GET calls in the experiment loop."""
    def __init__(self, publish, writer, payload, ros_now_ns):
        self.publish, self.writer, self.ros_now_ns = publish, writer, ros_now_ns
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.payload, self.error, self.thread = payload, None, None

    def set(self, payload):
        with self.lock: self.payload = dict(payload)

    def emit(self):
        with self.lock: payload = dict(self.payload)
        self.publish(json.dumps(payload, allow_nan=False))
        self.writer.write(dict(event='fault_control_published', receipt_monotonic=time.monotonic(),
                               receipt_ros_ns=self.ros_now_ns(), payload=payload))

    def start(self):
        def loop():
            while not self.stop.is_set():
                try: self.emit()
                except Exception as error:
                    self.error = error
                    self.stop.set()
                    return
                self.stop.wait(.1)
        self.thread = threading.Thread(target=loop, name='signal_trial_fault_only', daemon=True)
        self.thread.start()

    def close(self, payload):
        self.set(payload)
        self.stop.set()
        if self.thread is not None: self.thread.join(timeout=1.)
        self.emit()


def run(args):
    import rospy
    from std_msgs.msg import String
    from roslib.message import get_message_class
    from capture_traffic_light_phase_sequence import ModernNativeReader, native_readback_issue

    group_id = parse_group(args.group_id)
    groups, geometry = load_geometry(args.geometry)
    group = next(row for row in groups if row['group_id'] == group_id)
    stages = experiment_stages(group_id, args.experiment, getattr(args, 'yellow_durations_sec', None))
    moving_native = args.experiment == 'moving-native-yellow'
    first_yellow = args.experiment == 'first-observation-yellow'
    release_after_crossing = moving_native or first_yellow
    transport = native_transport(getattr(args, 'native_transport', 'persistent'))
    trigger_distance = moving_trigger_distance(getattr(args, 'trigger_front_distance_m', 8.))
    if not 0 < args.max_seconds <= 600 or not 0 <= args.front_axle_offset_m <= 6:
        raise ValueError('Bounded max_seconds0..600 and front offset0..6 required')
    output = args.output.expanduser().resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists(): raise FileExistsError('Fresh artifacts output directory required')
    output.mkdir(parents=True)
    metadata = dict(development_only=True, experiment=args.experiment, group=group, geometry=geometry,
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), stages=stages,
                    yellow_durations_sec=yellow_durations(
                        (3.,) if first_yellow and getattr(args, 'yellow_durations_sec', None) is None
                        else getattr(args, 'yellow_durations_sec', None)),
                    max_seconds=args.max_seconds, front_axle_offset_m=args.front_axle_offset_m,
                    trigger_front_distance_m=trigger_distance if moving_native else None,
                    terminal_release_after_finite_front_crossing=release_after_crossing,
                    terminal_crossing_tail_sec=TERMINAL_CROSSING_TAIL_SEC if release_after_crossing else None,
                    initial_arming_native_color=(controlled_go_state(group_id) if moving_native else
                                                initial_arming_phase(group_id, args.experiment)[0]),
                    first_observation_yellow_stress=first_yellow,
                    first_camera_observation_yellow_certified=False,
                    initial_yellow_hold_until_stationary=first_yellow,
                    canonical_signal_proxy=False if moving_native or first_yellow else 'owner may drop or change IDs',
                    native_multi_head_frame_wait_sec=MULTI_HEAD_FRAME_WAIT_SEC,
                    native_multi_head_order='SET head0; wait; SET head1; wait; independent GET each head',
                    native_transport=transport,
                    native_single_head_frame_wait_sec=(MULTI_HEAD_FRAME_WAIT_SEC if transport == 'persistent' else 0.),
                    signal_control=('Only selected native IDs; positive-only persistent gRPC, is_impulse=false, set_sibling=false'
                                    if transport == 'persistent' else
                                    'Explicit historical OfficialSender UDP impulse reproduction; stable phase not qualified'),
                    fault_control_topic=FAULT_TOPIC, native_rpc_retries=2, native_per_call_timeout_sec=.15,
                    limitations=['Non-natural phase-transition stress, NOT natural cycle timing or remaining-yellow estimation.',
                        'Native GET is API state, not proof of pixel/render state; send and verified actual are separately logged.',
                        'No vehicle control, no scenario load, no stack startup; owner watchdog must handle child error/abort.',
                        ('Terminal SET/GET ends only after the finite front crossing plus one second; fault heartbeat remains until owner SIGTERM.'
                         if release_after_crossing else 'The final stage repeatedly SETs/GETs controlled GO until owner SIGTERM, not a natural-program release.'),
                        ('First-observation-yellow holds Y/RY through the whole approach until the stationary gate, then starts the requested post-stop Y phase. Total Y can exceed that duration: intentional long-Y stress, not a natural period. Real first camera color requires independent source-stamped observation audit.'
                         if first_yellow else 'No first-yellow camera observation claim is made by the requested schedule.'),
                        'Cleanup leaves the last positive override untouched; manual natural restoration is required.',
                        'No mutation occurs until explicit execution; this tool is not an operational node.'])
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    writer, stop, lock, latest = JsonlWriter(output/'events.jsonl'), threading.Event(), threading.RLock(), {}
    rospy.init_node('traffic_light_policy_phase_trial', anonymous=True, disable_signals=True)
    deadline = time.monotonic()+args.max_seconds
    interrupted_signal = {'number': None}
    crossing_release = None
    def interrupted(signum, _frame):
        interrupted_signal['number'] = signum
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, interrupted)

    def callback(name):
        def receive(raw):
            mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
            with lock:
                if stop.is_set(): return
                try:
                    msgtype = raw._connection_header.get('type')
                    cls = get_message_class(msgtype)
                    if cls is None: raise ValueError('Unavailable ROS message type')
                    message = cls().deserialize(raw._buff)
                    source_ns = stamp_ns(getattr(getattr(message, 'header', None), 'stamp', None))
                    if source_ns is None: source_ns = stamp_ns(getattr(message, 'stamp', None))
                    row = dict(receipt_monotonic=mono, receipt_ros_ns=ros_ns, source_stamp_ns=source_ns)
                    if name == 'odom':
                        position, q, velocity = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
                        row.update(position_xyz=[position.x, position.y, position.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                                   velocity_xyz=[velocity.x, velocity.y, velocity.z], frame=message.header.frame_id)
                    elif name == 'gear': row['value'] = message.data
                    elif name == 'signal':
                        wanted = [g for g in message.traffic_light_groups if int(g.traffic_light_group_id) == group_id]
                        shape = 2 if group_id == 201326 else 1
                        row['authorized'] = len(wanted) == 1 and bool(wanted[0].elements) and all(
                            int(e.status) == 2 and int(e.color) != 0 and int(e.shape) != 0 for e in wanted[0].elements) and any(
                            int(e.color) == 3 and int(e.shape) == shape for e in wanted[0].elements)
                        row['signal_message'] = primitive(message)
                    else: row['control_message'] = primitive(message)
                    latest[name] = row
                    writer.write(dict(event='input_observation', input_name=name, message_type=msgtype, **row))
                    if name == 'odom' and crossing_release is not None:
                        crossing = crossing_release.update(row, mono)
                        if crossing is not None:
                            writer.write(dict(event='moving_front_finite_line_crossing', crossing=crossing,
                                              receipt_monotonic=mono, receipt_ros_ns=ros_ns,
                                              independent_native_qualification_required=True))
                except Exception as error:
                    latest.pop(name, None)
                    writer.write(dict(event='input_observation_invalid', input_name=name, error=type(error).__name__+':'+str(error),
                                      receipt_monotonic=mono, receipt_ros_ns=ros_ns))
        return receive

    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, callback(name), queue_size=5)
                     for name, topic in (('odom', args.odometry_topic), ('gear', args.gear_topic), ('ctrl', '/ctrl_cmd'))]
    if moving_native:
        subscriptions.append(rospy.Subscriber('/perception/traffic_light_recognition/traffic_signals',
                             rospy.AnyMsg, callback('signal'), queue_size=5))
    publisher = rospy.Publisher(FAULT_TOPIC, String, queue_size=1, latch=False)
    fault = FaultRepeater(lambda data: publisher.publish(String(data=data)), writer,
                          fault_payload('none', group_id, args.experiment, 'initializing'), lambda: rospy.Time.now().to_nsec())
    sender, reader, success, cleanup_errors = None, None, False, []
    last_requested_positive, last_verified_positive, last_native_observation = {}, {}, {}

    def emit(event, **fields):
        writer.write(dict(event=event, monotonic=time.monotonic(), ros_ns=rospy.Time.now().to_nsec(), **fields))

    def cleanup_emit(event, **fields):
        try: emit(event, **fields)
        except Exception as error:
            cleanup_errors.append('cleanup log unavailable: '+type(error).__name__+':'+str(error))

    def apply_and_read(color, stage):
        def send_one(native_id, requested):
            sent_at = time.monotonic()
            result = send_positive_signal(sender, native_id, requested)
            last_requested_positive[native_id] = dict(state=requested, requested_monotonic=sent_at, stage=stage)
            emit('native_signal_requested', stage=stage, native_id=native_id, requested_state=requested,
                 request_monotonic=sent_at, send=result)
            return result
        def read_one(native_id):
            request_at = time.monotonic()
            try:
                result = reader(native_id, deadline=deadline)
                issue = native_readback_issue(result, native_id)
                actual = result['observations'][0]['tl_color'] if issue is None else None
                last_native_observation[native_id] = dict(state=actual, verified=issue is None,
                    request_monotonic=request_at, response_monotonic=time.monotonic(), stage=stage)
                if issue is None:
                    last_verified_positive[native_id] = dict(last_native_observation[native_id])
                emit('native_signal_verified_actual', stage=stage, native_id=native_id, requested_state=color,
                     request_monotonic=request_at, response_monotonic=time.monotonic(), actual_state=actual,
                     verified=issue is None, matches_requested=issue is None and actual == color, readback=result,
                     validation_issue=issue)
                if issue is not None: raise TrialError('Native state/identity invalid: '+issue)
                return actual
            except Exception as error:
                last_native_observation[native_id] = dict(state=None, verified=False,
                    request_monotonic=request_at, response_monotonic=time.monotonic(), stage=stage,
                    error=type(error).__name__+':'+str(error))
                emit('native_read_error', stage=stage, native_id=native_id, requested_state=color,
                     request_monotonic=request_at, response_monotonic=time.monotonic(),
                     rpc_attempts=reader.last_rpc_attempts, error=type(error).__name__+':'+str(error))
                raise TrialError('Native observation failed; current color is unknown') from error
        states = apply_native_cycle(group['native_ids'], color, send_one, read_one, transport)
        return all(actual == color for actual in states.values())

    try:
        fault.start()
        fault.set(fault_payload('none', group_id, args.experiment, 'waiting_fresh_inputs'))
        def input_snapshot():
            with lock: return dict(latest)
        wait_for_preparation_inputs(
            input_snapshot, time.monotonic, lambda: rospy.Time.now().to_nsec(),
            lambda: stop.is_set() or rospy.is_shutdown(), stop.wait, deadline, emit,
            publisher_error=lambda: fault.error)
        sender = make_native_sender(transport)
        reader = ModernNativeReader(timeout_sec=.15, rpc_retries=2)
        hold, arming_started, armed = StartHold(), time.monotonic(), False
        if moving_native:
            green_count, last_stamp, green_confirmed = 0, None, None
            green_mismatch_since = None
            while not armed:
                if stop.is_set() or rospy.is_shutdown(): raise TrialError('Interrupted before moving trigger')
                if time.monotonic() >= deadline: raise TrialError('Moving trigger deadline exceeded')
                if fault.error is not None: raise TrialError('Fault-control publisher failed')
                matched = apply_and_read(controlled_go_state(group_id), 'moving_approach_authorized_green')
                if matched:
                    green_mismatch_since = None
                    if green_confirmed is None: green_confirmed = time.monotonic()
                else:
                    # Scene loading can reset a previously confirmed lamp while
                    # the owner still holds P. Bound each continuous mismatch,
                    # not total startup age; never qualify the trigger during it.
                    green_confirmed = None
                    if green_mismatch_since is None:
                        green_mismatch_since = time.monotonic()
                    elif time.monotonic()-green_mismatch_since >= APPLY_TIMEOUT_SEC:
                        raise TrialError('Moving approach native authorized phase not confirmed')
                with lock:
                    gate = moving_trigger_gate(latest, group, time.monotonic(), rospy.Time.now().to_nsec(),
                                               trigger_distance, args.front_axle_offset_m)
                source = gate['signal_source_stamp_ns']
                if matched and gate['route_authorized']:
                    if last_stamp is not None and source < last_stamp:
                        green_count, last_stamp = 0, None
                    if last_stamp is None or source > last_stamp:
                        green_count += 1
                        last_stamp = source
                else:
                    green_count, last_stamp = 0, None
                geometry = gate['geometry']
                if geometry['ready'] and geometry['front_distance_m'] <= .2:
                    raise TrialError('Moving trigger missed before verified authorization; no yellow issued')
                armed = gate['ready'] and green_count >= 3
                emit('moving_trigger_observation', gate=gate, verified_native_green=matched,
                     distinct_authorized_observations=green_count, armed=armed,
                     configured_trigger_front_distance_m=trigger_distance)
                if not armed: stop.wait(.05)
            emit('moving_native_yellow_triggered', gate=gate, native_green_confirmed_monotonic=green_confirmed,
                 distinct_authorized_observations=green_count,
                 real_camera_and_canonical_signal=True, synthetic_planner_signal=False)
            with lock:
                crossing_release = TerminalCrossingRelease(group, args.front_axle_offset_m)
                crossing_release.update(latest['odom'], time.monotonic())
        arming_color, arming_stage = initial_arming_phase(group_id, args.experiment)
        fault.set(fault_payload('none', group_id, args.experiment,
                                'moving_trigger_confirmed' if moving_native else arming_stage))
        red_mismatch_since = None
        while not armed:
            if stop.is_set() or rospy.is_shutdown(): raise TrialError('Interrupted before start gate')
            if time.monotonic() >= deadline: raise TrialError('Start gate global deadline exceeded')
            if fault.error is not None: raise TrialError('Fault-control publisher failed')
            matched = apply_and_read(arming_color, arming_stage)
            if matched:
                red_mismatch_since = None
            elif red_mismatch_since is None:
                red_mismatch_since = time.monotonic()
            elif time.monotonic()-red_mismatch_since >= APPLY_TIMEOUT_SEC:
                raise TrialError('Initial arming phase not confirmed')
            with lock:
                gate = start_gate(latest, group, time.monotonic(), rospy.Time.now().to_nsec(), args.front_axle_offset_m)
            armed = hold.update(gate['ready'] and matched, time.monotonic())
            emit('start_gate_observation', native_red_matches=matched, gate=gate,
                 hold_since_monotonic=hold.since, armed=armed)
            if not armed: stop.wait(.1)
        emit('experiment_start_gate_confirmed')
        if first_yellow:
            with lock:
                crossing_release = TerminalCrossingRelease(group, args.front_axle_offset_m)
                crossing_release.update(latest['odom'], time.monotonic())
        for stage in stages:
            if stage.get('action') in ('controlled_go_until_owner_teardown',
                                       'controlled_go_until_crossing_then_owner_teardown'):
                terminal = TerminalPositiveHold(stage['color'], time.monotonic())
                terminal_released = False
                fault.set(fault_payload('none', group_id, args.experiment, stage['stage']))
                emit('stage_requested', stage=stage)
                while True:
                    if stop.is_set() or rospy.is_shutdown():
                        if terminal_released:
                            if interrupted_signal['number'] != signal.SIGTERM:
                                raise TrialError('Released native program requires normal owner SIGTERM')
                        else:
                            terminal.finish_on_signal(interrupted_signal['number'])
                        emit('stage_complete', stage=stage, native_confirmed_monotonic=terminal.confirmed_at,
                             completion_basis=('SIGTERM_after_finite_crossing_and_native_SET_release'
                                               if terminal_released else 'SIGTERM_after_verified_terminal_controlled_GO'),
                             manual_natural_restore_required=True, natural_mode_restoration_certified=False)
                        break
                    if time.monotonic() >= deadline: raise TrialError('Terminal controlled-GO global deadline exceeded')
                    if fault.error is not None: raise TrialError('Fault-control publisher failed')
                    with lock:
                        fresh, reason = fresh_inputs(latest, time.monotonic(), rospy.Time.now().to_nsec())
                    if not fresh: raise TrialError('Terminal controlled-GO input invalid: '+reason)
                    if crossing_release is not None and crossing_release.ready(time.monotonic()):
                        if not terminal_released:
                            terminal_released = True
                            emit('terminal_native_SET_released_after_crossing', crossing=crossing_release.crossing,
                                 tail_sec=crossing_release.tail_sec, fault_heartbeat_continues=True,
                                 natural_mode_restoration_certified=False)
                        stop.wait(.1)
                        continue
                    matched = apply_and_read(stage['color'], stage['stage'])
                    if stop.is_set() or rospy.is_shutdown(): continue
                    if time.monotonic() >= deadline: raise TrialError('Terminal deadline exceeded during native read')
                    with lock:
                        fresh, reason = fresh_inputs(latest, time.monotonic(), rospy.Time.now().to_nsec())
                    if not fresh: raise TrialError('Terminal input became stale during native read: '+reason)
                    if terminal.update(time.monotonic(), matched):
                        emit('stage_native_confirmed', stage=stage, native_confirmed_monotonic=terminal.confirmed_at)
                        emit('terminal_controlled_go_confirmed', stage=stage,
                             native_confirmed_monotonic=terminal.confirmed_at,
                             positive_override_repeated_until_owner_teardown=True)
                    stop.wait(.1)
                continue
            phase = PhaseClock(stage, time.monotonic())
            fault.set(fault_payload(stage['fault'], group_id, args.experiment, stage['stage']))
            emit('stage_requested', stage=stage)
            while True:
                if stop.is_set() or rospy.is_shutdown(): raise TrialError('Interrupted during experiment')
                if time.monotonic() >= deadline: raise TrialError('Experiment global deadline exceeded')
                if fault.error is not None: raise TrialError('Fault-control publisher failed')
                with lock:
                    fresh, reason = fresh_inputs(latest, time.monotonic(), rospy.Time.now().to_nsec())
                if not fresh: raise TrialError('Active experiment input invalid: '+reason)
                matched = apply_and_read(stage['color'], stage['stage'])
                if time.monotonic() >= deadline: raise TrialError('Experiment deadline exceeded during native read')
                with lock:
                    fresh, reason = fresh_inputs(latest, time.monotonic(), rospy.Time.now().to_nsec())
                if not fresh: raise TrialError('Input became stale during native read: '+reason)
                was_confirmed = phase.confirmed_at
                done = phase.update(time.monotonic(), matched)
                if was_confirmed is None and phase.confirmed_at is not None:
                    emit('stage_native_confirmed', stage=stage, native_confirmed_monotonic=phase.confirmed_at)
                if done:
                    emit('stage_complete', stage=stage, native_confirmed_monotonic=phase.confirmed_at,
                         confirmed_elapsed_sec=time.monotonic()-phase.confirmed_at)
                    break
                stop.wait(.1)
        success = True
        emit('experiment_complete', official_violation_certified=False)
    except BaseException as error:
        emit('experiment_error', error=type(error).__name__+':'+str(error))
        raise
    finally:
        stop.set()
        try: fault.close(fault_payload('none', group_id, args.experiment, 'finished' if success else 'aborted'))
        except Exception as error:
            cleanup_errors.append('fault reset: '+str(error))
            cleanup_emit('fault_reset_error', error=str(error))
        # Deliberately no native SET in cleanup on success/error/interrupt.
        # Abrupt RED during motion and negative reset states are both excluded.
        for name, resource in (('native_reader', reader), ('official_sender', sender)):
            if resource is not None:
                try: resource.close()
                except Exception as error:
                    cleanup_errors.append(name+':'+str(error))
                    cleanup_emit('resource_cleanup_error', resource=name, error=str(error))
        for subscription in subscriptions:
            try: subscription.unregister()
            except Exception as error: cleanup_errors.append('unsubscribe: '+str(error))
        with lock:
            cleanup_emit('cleanup_complete', experiment_completed=success, cleanup_errors=cleanup_errors,
                         last_requested_positive=last_requested_positive,
                         last_verified_positive=last_verified_positive,
                         last_native_observation=last_native_observation,
                         native_SET_in_cleanup=False, current_native_state_certified=False,
                         manual_natural_restore_required=True, natural_mode_restoration_certified=False)
            (output/'summary.json').write_text(json.dumps(dict(experiment_completed=success,
                cleanup_errors=cleanup_errors, group_id=group_id, experiment=args.experiment,
                native_transport=transport,
                last_requested_positive=last_requested_positive, last_verified_positive=last_verified_positive,
                last_native_observation=last_native_observation, native_SET_in_cleanup=False,
                manual_natural_restore_required=True, current_native_state_certified=False,
                natural_mode_restoration_certified=False, official_violation_certified=False), indent=2)+'\n')
        writer.close()
        if cleanup_errors and success: raise TrialError('Cleanup failed: '+';'.join(cleanup_errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--group-id', required=True)
    parser.add_argument('--experiment', choices=('stopped-yellow', 'missing-signal', 'moving-native-yellow',
                                                'first-observation-yellow'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=120.)
    parser.add_argument('--front-axle-offset-m', type=float, default=3.)
    parser.add_argument('--trigger-front-distance-m', type=float, default=8.)
    parser.add_argument('--native-transport', choices=('persistent', 'udp'), default='persistent',
                        help='Persistent gRPC by default; udp only reproduces unqualified historical impulse trials')
    parser.add_argument('--odometry-topic', default='/localization/kinematic_state')
    parser.add_argument('--gear-topic', default='/vehicle/status/gear')
    parser.add_argument('--yellow-durations-sec', type=parse_yellow_durations,
                        help='Comma-separated positive durations <=10 sec; default 1,3,5')
    run(parser.parse_args())


if __name__ == '__main__': main()
