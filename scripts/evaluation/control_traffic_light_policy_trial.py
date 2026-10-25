#!/usr/bin/env python3
"""Development-only signal stress sequence; never commands a vehicle/stack.

Only the selected A/B/C/D native IDs receive official UDP signal overrides.
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
                                             load_geometry, primitive, rotate, stamp_ns)

FAULT_TOPIC = '/evaluation/traffic_light_policy/fault'
FRESH_SEC = 1.
START_HOLD_SEC = 1.
APPLY_TIMEOUT_SEC = 2.


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


def experiment_stages(group_id, experiment, yellow_durations_sec=None):
    go = controlled_go_state(group_id)
    durations = yellow_durations(yellow_durations_sec)
    yellow = 5 if group_id == 201326 else 4
    terminal = dict(stage='terminal_controlled_go', action='controlled_go_until_owner_teardown',
                    color=go, seconds=None, fault='none')
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


def coordinated_native_cycle(native_ids, state, send, read):
    """SET every selected head before GET of any head; this is not atomic GT.

    Geometry validation independently pins exact mission IDs. Per-head send/read
    observations must remain separate; no synthetic common color is produced.
    """
    if (not isinstance(native_ids, (list, tuple)) or not native_ids or
            any(not isinstance(item, str) or not item.strip() for item in native_ids) or
            len(set(native_ids)) != len(native_ids)):
        raise ValueError('Unique explicit native IDs are required')
    for native_id in native_ids:
        send_positive_signal(send, native_id, state)
    return {native_id: read(native_id) for native_id in native_ids}


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
    from capture_traffic_light_phase_sequence import OfficialSender, ModernNativeReader, native_readback_issue

    group_id = parse_group(args.group_id)
    groups, geometry = load_geometry(args.geometry)
    group = next(row for row in groups if row['group_id'] == group_id)
    stages = experiment_stages(group_id, args.experiment, getattr(args, 'yellow_durations_sec', None))
    if not 0 < args.max_seconds <= 600 or not 0 <= args.front_axle_offset_m <= 6:
        raise ValueError('Bounded max_seconds0..600 and front offset0..6 required')
    output = args.output.expanduser().resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists(): raise FileExistsError('Fresh artifacts output directory required')
    output.mkdir(parents=True)
    metadata = dict(development_only=True, experiment=args.experiment, group=group, geometry=geometry,
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), stages=stages,
                    yellow_durations_sec=yellow_durations(getattr(args, 'yellow_durations_sec', None)),
                    max_seconds=args.max_seconds, front_axle_offset_m=args.front_axle_offset_m,
                    signal_control='Only selected native IDs; positive-only OfficialSender UDP; no cleanup signal writes',
                    fault_control_topic=FAULT_TOPIC, native_rpc_retries=2, native_per_call_timeout_sec=.15,
                    limitations=['Non-natural phase-transition stress, NOT natural cycle timing or remaining-yellow estimation.',
                        'Native GET is API state, not proof of pixel/render state; send and verified actual are separately logged.',
                        'No vehicle control, no scenario load, no stack startup; owner watchdog must handle child error/abort.',
                        'The final stage repeatedly SETs/GETs controlled A16/D33 until owner SIGTERM, not a natural-program release.',
                        'Cleanup leaves the last positive override untouched; manual natural restoration is required.',
                        'No mutation occurs until explicit execution; this tool is not an operational node.'])
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    writer, stop, lock, latest = JsonlWriter(output/'events.jsonl'), threading.Event(), threading.RLock(), {}
    rospy.init_node('traffic_light_policy_phase_trial', anonymous=True, disable_signals=True)
    deadline = time.monotonic()+args.max_seconds
    interrupted_signal = {'number': None}
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
                    else: row['control_message'] = primitive(message)
                    latest[name] = row
                    writer.write(dict(event='input_observation', input_name=name, message_type=msgtype, **row))
                except Exception as error:
                    latest.pop(name, None)
                    writer.write(dict(event='input_observation_invalid', input_name=name, error=type(error).__name__+':'+str(error),
                                      receipt_monotonic=mono, receipt_ros_ns=ros_ns))
        return receive

    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, callback(name), queue_size=5)
                     for name, topic in (('odom', args.odometry_topic), ('gear', args.gear_topic), ('ctrl', '/ctrl_cmd'))]
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
        states = coordinated_native_cycle(group['native_ids'], color, send_one, read_one)
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
        sender = OfficialSender()
        reader = ModernNativeReader(timeout_sec=.15, rpc_retries=2)
        hold, arming_started, armed = StartHold(), time.monotonic(), False
        fault.set(fault_payload('none', group_id, args.experiment, 'arming_red'))
        while not armed:
            if stop.is_set() or rospy.is_shutdown(): raise TrialError('Interrupted before start gate')
            if time.monotonic() >= deadline: raise TrialError('Start gate global deadline exceeded')
            if fault.error is not None: raise TrialError('Fault-control publisher failed')
            matched = apply_and_read(1, 'arming_red')
            if not matched and time.monotonic()-arming_started >= APPLY_TIMEOUT_SEC:
                raise TrialError('Initial red phase not confirmed')
            with lock:
                gate = start_gate(latest, group, time.monotonic(), rospy.Time.now().to_nsec(), args.front_axle_offset_m)
            armed = hold.update(gate['ready'] and matched, time.monotonic())
            emit('start_gate_observation', native_red_matches=matched, gate=gate,
                 hold_since_monotonic=hold.since, armed=armed)
            if not armed: stop.wait(.1)
        emit('experiment_start_gate_confirmed')
        for stage in stages:
            if stage.get('action') == 'controlled_go_until_owner_teardown':
                terminal = TerminalPositiveHold(stage['color'], time.monotonic())
                fault.set(fault_payload('none', group_id, args.experiment, stage['stage']))
                emit('stage_requested', stage=stage)
                while True:
                    if stop.is_set() or rospy.is_shutdown():
                        terminal.finish_on_signal(interrupted_signal['number'])
                        emit('stage_complete', stage=stage, native_confirmed_monotonic=terminal.confirmed_at,
                             completion_basis='SIGTERM_after_verified_terminal_controlled_GO',
                             manual_natural_restore_required=True, natural_mode_restoration_certified=False)
                        break
                    if time.monotonic() >= deadline: raise TrialError('Terminal controlled-GO global deadline exceeded')
                    if fault.error is not None: raise TrialError('Fault-control publisher failed')
                    with lock:
                        fresh, reason = fresh_inputs(latest, time.monotonic(), rospy.Time.now().to_nsec())
                    if not fresh: raise TrialError('Terminal controlled-GO input invalid: '+reason)
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
    parser.add_argument('--experiment', choices=('stopped-yellow', 'missing-signal'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=120.)
    parser.add_argument('--front-axle-offset-m', type=float, default=3.)
    parser.add_argument('--odometry-topic', default='/localization/kinematic_state')
    parser.add_argument('--gear-topic', default='/vehicle/status/gear')
    parser.add_argument('--yellow-durations-sec', type=parse_yellow_durations,
                        help='Comma-separated positive durations <=10 sec; default 1,3,5')
    run(parser.parse_args())


if __name__ == '__main__': main()
