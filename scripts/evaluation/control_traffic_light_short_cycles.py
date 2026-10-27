#!/usr/bin/env python3
"""Development-only native-lamp short cycles for a full-course camera trial.

Only exact mission heads receive official persistent gRPC SET commands. There is
no vehicle command or ROS signal publisher: perception must read real images.
Requested schedules and native GET observations remain separate evidence.
SIGTERM stops sending without inventing a safe/restored final lamp state.

LIMITATION (2026-10-27 live audit): UDP SET does not establish a persistent
manual mode. Run03's independent GET observed a brief real green inside a
requested red phase. These runs are impulse/control-interference stress,
not qualified stable short programs. Do not attribute every jerk to the
camera/planner or use requested phases as ground truth. Qualify persistent
control separately before further stable-cycle driving comparisons.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

from observe_traffic_light_compliance import GROUP_IDS, JsonlWriter, ROOT, finite, stamp_ns
from control_traffic_light_policy_trial import (MULTI_HEAD_FRAME_WAIT_SEC, coordinated_native_cycle,
                                               fresh_inputs, TrialError)
from probe_morai_traffic_light_gt import (_fields, encode_target, encode_varint,
                                        validate_endpoint, validate_timeout)

NATURAL_CYCLES = {201351: (16, 4, 1), 201365: (16, 4, 1),
                  201290: (48, 4, 1), 201326: (16, 4, 33, 5, 1)}
MAX_OWNER_DURATION_SEC = 1800.
PERSISTENT_SET_METHOD = '/morai_sim_api.infrastructure.Infrastructure/SetTrafficLightState'
PERSISTENT_SET_SOURCE = '/home/luv/Applications/WeatherTimeChanger_Linux'
PERSISTENT_SET_SOURCE_SHA256 = '8c47f48a8d54b79b657920a661f582bd587c9f9065b6b55c9e911d42a0369609'


def persistent_request(native_id, color):
    """Inspected TrafficLightStateParam; explicit false impulse/sibling bits.

    WeatherTimeChanger's simulation_world.set_traffic_light_info documents
    impulse=False as permanent and sibling=False as this physical head only.
    The current server/render behavior is independently checked, never inferred
    solely from a successful Result response.
    """
    group_id = next((group for group, ids in GROUP_IDS.items() if native_id in ids), None)
    if group_id is None or type(color) is not int or color not in NATURAL_CYCLES[group_id]:
        raise ValueError('Exact mission physical ID and its positive natural phase required')
    identifier = encode_target(native_id)  # ObjectIdentifier.value:string=1
    info = b'\x12' + encode_varint(len(identifier)) + identifier + b'\x18' + encode_varint(color)
    # info:message=1, is_impulse:bool=2(false), set_sibling:bool=3(false).
    # Explicit false wire values are valid proto3; serializer omission is equal.
    return b'\x0a' + encode_varint(len(info)) + info + b'\x10\x00\x18\x00'


def persistent_result(payload):
    result = dict(status=0, description='', custom_message='')
    for field, wire, value in _fields(payload):
        if field == 1:
            if wire != 0:
                raise ValueError('Result.status must be a varint')
            result['status'] = value
        elif field in (2, 3):
            if wire != 2:
                raise ValueError('Result descriptions must be strings')
            result['description' if field == 2 else 'custom_message'] = value.decode('utf-8')
    if result['status'] != 1:
        raise TrialError('Persistent native SET rejected: ' + str(result))
    return result


class PersistentNativeSender:
    """Narrow official permanent-state RPC, no vehicle or sibling operations."""
    def __init__(self, rpc_call=None, endpoint='127.0.0.1:7789', timeout_sec=.5):
        self.endpoint = validate_endpoint(endpoint)
        self.timeout_sec = validate_timeout(timeout_sec)
        self.channel = None
        if rpc_call is None:
            import grpc
            self.channel = grpc.insecure_channel(self.endpoint)
            unary = self.channel.unary_unary(PERSISTENT_SET_METHOD,
                request_serializer=lambda value: value, response_deserializer=lambda value: value)
            self.rpc_call = lambda path, request, timeout: unary(request, timeout=timeout)
        else:
            self.rpc_call = rpc_call

    def __call__(self, native_id, color):
        request = persistent_request(native_id, color)
        response = self.rpc_call(PERSISTENT_SET_METHOD, request, self.timeout_sec)
        result = persistent_result(response)
        return dict(rpc_set_success=True, method=PERSISTENT_SET_METHOD,
                    is_impulse=False, set_sibling=False, request_bytes=len(request),
                    request_sha256=hashlib.sha256(request).hexdigest(), result=result,
                    simulator_applied_confirmed=False)

    def close(self):
        if self.channel is not None:
            self.channel.close()


def validate_max_seconds(value):
    """Match the owning wrapper's preparation+case bound, not case time alone."""
    value = finite(value)
    if not 0 < value <= MAX_OWNER_DURATION_SEC:
        raise ValueError('max_seconds must be within (0,1800]')
    return value


def paced_native_cycle(native_ids, color, send, read, frame_wait=time.sleep):
    """Give EVERY physical SET one frame interval before GET/next SET.

    The shared helper already paces multi-head C. Its legacy single-head path
    intentionally did not; this multi-group scheduler must pace A/B/D too so
    consecutive controllers cannot overwrite one simulator-frame command slot.
    """
    def paced_single_send(native_id, state):
        result = send(native_id, state)
        frame_wait(MULTI_HEAD_FRAME_WAIT_SEC)
        return result
    selected_send = paced_single_send if len(native_ids) == 1 else send
    return coordinated_native_cycle(native_ids, color, selected_send, read, frame_wait=frame_wait)


def validate_plan(plan):
    if not isinstance(plan, dict) or plan.get('schema_version') != 1:
        raise ValueError('Explicit schema_version 1 required')
    groups = plan.get('groups')
    if not isinstance(groups, list) or len(groups) != 4:
        raise ValueError('Exactly four mission groups required')
    seen = set()
    for group in groups:
        if not isinstance(group, dict):
            raise ValueError('Group must be an object')
        group_id = group.get('group_id')
        if type(group_id) is not int or group_id not in GROUP_IDS or group_id in seen:
            raise ValueError('Exact unique mission group IDs required; old D-1 excluded')
        seen.add(group_id)
        if group.get('native_ids') != list(GROUP_IDS[group_id]):
            raise ValueError('Exact ordered mission native IDs required, including both C heads')
        offset = finite(group.get('phase_offset_sec', 0))
        if not 0 <= offset <= 120:
            raise ValueError('phase_offset_sec must be bounded 0..120')
        phases = group.get('phases')
        expected = NATURAL_CYCLES[group_id]
        if not isinstance(phases, list) or len(phases) != len(expected):
            raise ValueError('Complete natural-combination cycle required')
        colors = tuple(row.get('color') for row in phases if isinstance(row, dict))
        if any(type(value) is not int for value in colors) or colors not in tuple(
                expected[index:] + expected[:index] for index in range(len(expected))):
            raise ValueError('Preserve each mission natural phase order and combinations')
        total = 0.
        for phase in phases:
            seconds = finite(phase.get('seconds'))
            if not 1 <= seconds <= 30:
                raise ValueError('Each phase must be within 1..30 seconds')
            if phase['color'] in (4, 5) and seconds > 5:
                raise ValueError('Yellow stress phases must be within 1..5 seconds')
            total += seconds
        if total > 120:
            raise ValueError('Total program must be at most 120 seconds')
    return copy.deepcopy(plan)


def scheduled_phase(group, elapsed_sec):
    """Pure fixed-anchor phase selection; calling it never restarts the clock."""
    elapsed_sec = finite(elapsed_sec)
    if elapsed_sec < 0:
        raise ValueError('Monotonic clock regressed')
    phases = group['phases']
    total = sum(float(row['seconds']) for row in phases)
    absolute = elapsed_sec + float(group.get('phase_offset_sec', 0))
    cycle_index = math.floor(absolute / total)
    within = absolute - cycle_index * total
    beginning = 0.
    for index, phase in enumerate(phases):
        end = beginning + float(phase['seconds'])
        if within < end or index == len(phases) - 1:
            return dict(color=phase['color'], phase_index=index, cycle_index=cycle_index,
                        phase_elapsed_sec=within - beginning,
                        phase_remaining_sec=max(0., end - within),
                        schedule_elapsed_sec=elapsed_sec)
        beginning = end
    raise AssertionError('Unreachable phase')


class MismatchGuard:
    def __init__(self, timeout_sec=2.):
        self.timeout_sec = finite(timeout_sec)
        if self.timeout_sec <= 0:
            raise ValueError('Positive mismatch bound required')
        self.phase_key, self.since, self.last_now = None, None, None

    def update(self, now, phase_key, matches):
        now = finite(now)
        if self.last_now is not None and now < self.last_now:
            raise TrialError('Native comparison clock regressed')
        self.last_now = now
        if phase_key != self.phase_key:
            self.phase_key, self.since = phase_key, None
        if matches:
            self.since = None
        elif self.since is None:
            self.since = now
        elif now - self.since >= self.timeout_sec:
            raise TrialError('Native phase mismatch persisted for at least two seconds')


def input_gate(latest, now, ros_ns):
    fresh, reason = fresh_inputs(latest, now, ros_ns)
    if not fresh:
        return False, reason
    if latest['gear'].get('value') != 4:
        return False, 'gear_not_D4'
    if latest['odom'].get('frame', '').strip('/') != 'map':
        return False, 'odom_not_map'
    return True, 'fresh_D4_inputs_no_green_or_stationary_requirement'


def run(args):
    from capture_traffic_light_phase_sequence import ModernNativeReader, native_readback_issue
    import rospy
    from roslib.message import get_message_class

    plan_bytes = args.plan.expanduser().resolve(strict=True).read_bytes()
    plan = validate_plan(json.loads(plan_bytes))
    maximum = validate_max_seconds(args.max_seconds)
    requested = args.output.expanduser()
    output = requested.resolve()
    output.relative_to(ROOT / 'artifacts/evaluation')
    if output == ROOT / 'artifacts/evaluation' or requested.exists() or requested.is_symlink() or output.exists():
        raise FileExistsError('Fresh child artifact output required')
    output.mkdir(parents=True)
    (output / 'plan.json').write_bytes(plan_bytes)
    (output / 'metadata.json').write_text(json.dumps(dict(
        development_only=True, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        plan_sha256=hashlib.sha256(plan_bytes).hexdigest(), max_seconds=maximum,
        canonical_signal_publisher=False, vehicle_commands=False,
        native_signal_SET=True, controller_program_is_natural=False,
        native_SET_transport='persistent_gRPC', is_impulse=False, set_sibling=False,
        native_SET_schema_source=PERSISTENT_SET_SOURCE,
        native_SET_schema_source_sha256=PERSISTENT_SET_SOURCE_SHA256,
        native_GET_is_rendered_pixel_GT=False, monotonic_anchor_once=True,
        every_physical_SET_frame_wait_sec=MULTI_HEAD_FRAME_WAIT_SEC,
        manual_natural_restore_required=True,
        limitations=['Artificial shortened native lamp schedule, not an installed natural controller program.',
                     'Native readback is not proof of render timing or atomic C-head transition.',
                     'Cleanup sends no lamp state; restart/explicit natural restoration remains necessary.']), indent=2) + '\n')
    writer = JsonlWriter(output / 'events.jsonl')
    latest, lock, stop = {}, threading.RLock(), threading.Event()
    shutdown_signal = {'number': None}
    rospy.init_node('traffic_light_short_cycles', anonymous=True, disable_signals=True)
    deadline = time.monotonic() + maximum
    old_handlers = {}

    def emit(event, **fields):
        writer.write(dict(event=event, monotonic=time.monotonic(), ros_ns=rospy.Time.now().to_nsec(), **fields))

    def interrupted(signum, _frame):
        shutdown_signal['number'] = signum
        stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, interrupted)

    def snapshot():
        with lock:
            return copy.deepcopy(latest)

    def receive(name):
        def callback(raw):
            try:
                cls = get_message_class(raw._connection_header.get('type'))
                if cls is None:
                    raise ValueError('Unavailable ROS type')
                message = cls().deserialize(raw._buff)
                source_ns = stamp_ns(getattr(getattr(message, 'header', None), 'stamp', None))
                if source_ns is None:
                    source_ns = stamp_ns(getattr(message, 'stamp', None))
                row = dict(receipt_monotonic=time.monotonic(), source_stamp_ns=source_ns)
                if name == 'odom':
                    p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
                    row.update(position_xyz=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                               velocity_xyz=[v.x, v.y, v.z], frame=message.header.frame_id)
                elif name == 'gear':
                    row['value'] = message.data
                with lock:
                    latest[name] = row
            except Exception as error:
                with lock:
                    latest.pop(name, None)
                emit('invalid_input', input_name=name, error=type(error).__name__ + ':' + str(error))
        return callback

    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, receive(name), queue_size=5)
                     for name, topic in (('odom', '/localization/kinematic_state'),
                                         ('gear', '/vehicle/status/gear'), ('ctrl', '/ctrl_cmd'))]
    sender = reader = None
    success, error_text, anchor = False, None, None
    last_sent = {}
    guards = {row['group_id']: MismatchGuard() for row in plan['groups']}

    def verify_inputs():
        valid, reason = input_gate(snapshot(), time.monotonic(), rospy.Time.now().to_nsec())
        if not valid:
            raise TrialError('Input freshness/gear invalid after arming: ' + reason)

    try:
        while not stop.is_set() and not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                raise TrialError('Fresh D4 preparation deadline exceeded; no native SET sent')
            valid, reason = input_gate(snapshot(), time.monotonic(), rospy.Time.now().to_nsec())
            emit('preparation_input_observation', ready=valid, reason=reason)
            if valid:
                break
            stop.wait(.1)
        if stop.is_set() or rospy.is_shutdown():
            success = shutdown_signal['number'] == signal.SIGTERM
            return 0 if success else 130
        sender = PersistentNativeSender()
        reader = ModernNativeReader(timeout_sec=.15, rpc_retries=2)
        anchor = time.monotonic()
        emit('schedule_started', anchor_monotonic=anchor, verified_inputs=snapshot())
        while not stop.is_set() and not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                raise TrialError('Short-cycle global deadline exceeded before owner teardown')
            for group in plan['groups']:
                if stop.is_set():
                    break
                verify_inputs()
                desired = scheduled_phase(group, time.monotonic() - anchor)
                group_id, color = group['group_id'], desired['color']
                phase_key = (desired['cycle_index'], desired['phase_index'])
                emit('desired_phase', group_id=group_id, anchor_monotonic=anchor, **desired)

                def send_one(native_id, state):
                    if stop.is_set():
                        raise InterruptedError('Owner teardown before native SET')
                    verify_inputs()
                    result = sender(native_id, state)
                    last_sent[native_id] = dict(color=state, monotonic=time.monotonic())
                    emit('native_signal_requested', group_id=group_id, native_id=native_id,
                         requested_state=state, send=result, schedule=desired)
                    return result

                def read_one(native_id):
                    started = time.monotonic()
                    try:
                        result = reader(native_id, deadline=deadline)
                        issue = native_readback_issue(result, native_id)
                        actual = result['observations'][0]['tl_color'] if issue is None else None
                        emit('native_signal_verified_actual', group_id=group_id, native_id=native_id,
                             requested_state=color, actual_state=actual, verified=issue is None,
                             matches_requested=issue is None and actual == color,
                             request_monotonic=started, response_monotonic=time.monotonic(),
                             readback=result, validation_issue=issue, schedule=desired)
                        return actual
                    except Exception as error:
                        emit('native_read_error', group_id=group_id, native_id=native_id,
                             request_monotonic=started, response_monotonic=time.monotonic(),
                             error=type(error).__name__ + ':' + str(error), schedule=desired)
                        return None

                states = paced_native_cycle(group['native_ids'], color, send_one, read_one,
                                             frame_wait=lambda seconds: stop.wait(seconds))
                guards[group_id].update(time.monotonic(), phase_key,
                                        all(value == color for value in states.values()))
            stop.wait(.1)
        success = shutdown_signal['number'] == signal.SIGTERM
        if not success:
            raise TrialError('Unrequested ROS shutdown or non-SIGTERM interruption')
        return 0
    except InterruptedError:
        success = shutdown_signal['number'] == signal.SIGTERM
        if success:
            return 0
        raise
    except BaseException as error:
        error_text = type(error).__name__ + ':' + str(error)
        emit('short_cycle_error', error=error_text)
        raise
    finally:
        stop.set()
        cleanup_errors = []
        for subscription in subscriptions:
            try:
                subscription.unregister()
            except Exception as error:
                cleanup_errors.append(str(error))
        for resource in (reader, sender):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    cleanup_errors.append(str(error))
        summary = dict(successful_owner_teardown=success, shutdown_signal=shutdown_signal['number'],
                       error=error_text, anchor_monotonic=anchor, last_requested_positive=last_sent,
                       native_SET_in_cleanup=False, natural_mode_restoration_certified=False,
                       manual_natural_restore_required=True, cleanup_errors=cleanup_errors)
        emit('cleanup_complete', **summary)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        writer.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=600.)
    return run(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
