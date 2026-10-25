#!/usr/bin/env python3
"""Development-only controlled-GO departure -> real RED + signal-drop stress.

Never publishes GT colors/planner signal messages or vehicle commands. Official
UDP controls only selected A/B/C/D native signals; ROS emits only the existing
explicit fault-control payload. An independent owner/watchdog must stop/park the
vehicle. Script completion is NOT a signal-compliance PASS.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

from control_traffic_light_policy_trial import (
    APPLY_TIMEOUT_SEC, FAULT_TOPIC, ROOT, FaultRepeater, JsonlWriter, PhaseClock,
    StartHold, TerminalPositiveHold, TrialError, fault_payload, finite, fresh_inputs, front_stop_geometry,
    load_geometry, parse_group, primitive, stamp_ns, start_gate,
    wait_for_preparation_inputs, send_positive_signal, controlled_go_state,
    coordinated_native_cycle, parse_yellow_durations)

EXPERIMENT = 'green-loss'
MIN_DEPARTURE_SPEED_MPS = .15
MAX_DEPARTURE_SPEED_MPS = .4
MIN_FRONT_DISTANCE_M = .4


class TrialNotApplicable(TrialError):
    """The narrow departure window was not observed, not a compliance result."""


def native_route_permission(group_id, states, native_ids):
    expected = controlled_go_state(group_id)
    if not isinstance(states, dict) or set(states) != set(native_ids):
        raise TrialError('Exact selected native identities required')
    if any(type(value) is not int or value <= 0 for value in states.values()):
        raise TrialError('Explicit positive native color required')
    # A/B are straight, C is combined G+GL, D is protected left. Y/RY is not new GO.
    allowed = {32, 33} if group_id == 201326 else {expected}
    return all(value in allowed for value in states.values())


def controlled_go_matches(group_id, states, native_ids):
    return (native_route_permission(group_id, states, native_ids) and
            all(value == controlled_go_state(group_id) for value in states.values()))


def forward_control(message):
    if not isinstance(message, dict):
        raise TrialError('CtrlCmd object missing')
    kind = message.get('longlCmdType')
    if type(kind) is not int or kind not in (1, 2, 3):
        raise TrialError('Known CtrlCmd.longlCmdType required')
    brake = finite(message.get('brake'))
    requested = finite(message.get({1: 'accel', 2: 'velocity', 3: 'acceleration'}[kind]))
    if brake < 0 or kind == 1 and requested < 0:
        raise TrialError('Malformed CtrlCmd brake/accel')
    return brake < .1 and requested > 0


def departure_gate(latest, group, now, ros_ns, offset=3.):
    fresh, reason = fresh_inputs(latest, now, ros_ns)
    if not fresh:
        raise TrialError('Active experiment input invalid: '+reason)
    if latest['gear']['value'] != 4:
        raise TrialError('Departure requires fresh D4 gear')
    speed = math.sqrt(sum(finite(value)**2 for value in latest['odom']['velocity_xyz']))
    geometry = front_stop_geometry(latest['odom'], group, offset)
    if not geometry['ready']:
        return dict(ready=False, not_applicable=True, reason='front_stop_geometry_outside_departure_window',
                    speed_mps=speed, geometry=geometry)
    distance = min(row['forward_distance_m'] for row in geometry['matches'])
    forward = forward_control(latest['ctrl']['control_message'])
    too_late = speed > MAX_DEPARTURE_SPEED_MPS or distance <= MIN_FRONT_DISTANCE_M
    return dict(ready=forward and MIN_DEPARTURE_SPEED_MPS <= speed <= MAX_DEPARTURE_SPEED_MPS
                      and distance > MIN_FRONT_DISTANCE_M,
                not_applicable=too_late, reason='departure_window_missed' if too_late else
                    'departing_forward' if forward and speed >= MIN_DEPARTURE_SPEED_MPS else 'waiting_actual_departure',
                speed_mps=speed, forward_ctrl=forward, front_distance_m=distance, geometry=geometry)


def execute_sequence(group, snapshot, monotonic, ros_now_ns, wait, stopped, deadline,
                     set_fault, apply_red, apply_go, emit,
                     publisher_error=lambda: None, front_axle_offset_m=3., termination_signal=lambda: None):
    """Dependency-injected sequence; tests use no ROS/native/network resources."""
    group_id = group['group_id']

    def check(stage, go_wait=False):
        if stopped():
            raise TrialError('Interrupted during '+stage)
        if monotonic() >= deadline:
            if go_wait:
                raise TrialNotApplicable('Controlled GO/departure not observed before global deadline')
            raise TrialError('Global deadline exceeded during '+stage)
        if publisher_error() is not None:
            raise TrialError('Fault-control publisher failed')
        fresh, reason = fresh_inputs(snapshot(), monotonic(), ros_now_ns())
        if not fresh:
            raise TrialError('Active experiment input invalid: '+reason)

    hold, arming_started = StartHold(), monotonic()
    set_fault('none', 'arming_red', False)
    while True:
        check('arming_red')
        matched = apply_red('arming_red')
        if not matched and monotonic()-arming_started >= APPLY_TIMEOUT_SEC:
            raise TrialError('Initial red phase not confirmed')
        check('arming_red_after_read')
        gate = start_gate(snapshot(), group, monotonic(), ros_now_ns(), front_axle_offset_m)
        armed = hold.update(gate['ready'] and matched, monotonic())
        emit('start_gate_observation', native_red_matches=matched, gate=gate,
             hold_since_monotonic=hold.since, armed=armed)
        if armed:
            break
        wait(.05)
    emit('experiment_start_gate_confirmed')
    # Natural-mode GET returns UNDEFINED (-1) on this inspected API, so it
    # cannot verify a naturally rendered phase. Use a condition-driven official
    # GO override with explicit positive GET; never infer a color from -1.
    set_fault('none', 'arming_controlled_go', False)
    while True:
        check('arming_controlled_go', go_wait=True)
        states = apply_go('arming_controlled_go')
        check('departure_after_native_read', go_wait=True)
        permitted = controlled_go_matches(group_id, states, group['native_ids'])
        gate = departure_gate(snapshot(), group, monotonic(), ros_now_ns(), front_axle_offset_m)
        emit('controlled_departure_observation', native_states=states,
             requested_go_state=controlled_go_state(group_id),
             exact_controlled_go_confirmed=permitted, gate=gate)
        if gate['not_applicable']:
            raise TrialNotApplicable(gate['reason'])
        if permitted and gate['ready']:
            # Synchronous emission BEFORE RED SET/GET; never wait for the next
            # background heartbeat. Actual proxy application still needs its log.
            set_fault('drop', 'red_drop_after_controlled_departure', True)
            emit('departure_fault_trigger_requested', native_states=states, gate=gate,
                 actual_proxy_drop_certified=False)
            break
        wait(.02)
    for stage in (dict(stage='red_drop_after_controlled_departure', seconds=3., fault='drop'),
                  dict(stage='red_restored', seconds=2., fault='none')):
        set_fault(stage['fault'], stage['stage'], True)
        phase = PhaseClock(stage, monotonic())
        emit('stage_requested', stage=dict(stage, color=1))
        while True:
            check(stage['stage'])
            matched = apply_red(stage['stage'])
            check(stage['stage']+'_after_native_read')
            if phase.update(monotonic(), matched):
                emit('stage_complete', stage=stage, native_confirmed_monotonic=phase.confirmed_at,
                     confirmed_elapsed_sec=monotonic()-phase.confirmed_at)
                break
            wait(.05)
    # Positive-only controlled terminal state. The owner must explicitly SIGTERM
    # after confirmation; no native writes occur in finally on success or error.
    stage = 'terminal_controlled_go'
    set_fault('none', stage, True)
    terminal = TerminalPositiveHold(controlled_go_state(group_id), monotonic())
    emit('stage_requested', stage=dict(stage=stage, color=controlled_go_state(group_id),
                                      seconds=None, action='controlled_go_until_owner_teardown'))
    while True:
        if stopped():
            terminal.finish_on_signal(termination_signal())
            emit('stage_complete', stage=dict(stage=stage, color=controlled_go_state(group_id)),
                 native_confirmed_monotonic=terminal.confirmed_at, owner_teardown_signal=termination_signal(),
                 completion_basis='SIGTERM_after_verified_terminal_controlled_GO')
            break
        check(stage)
        states = apply_go(stage)
        if stopped():
            terminal.finish_on_signal(termination_signal())
            emit('stage_complete', stage=dict(stage=stage, color=controlled_go_state(group_id)),
                 native_confirmed_monotonic=terminal.confirmed_at, owner_teardown_signal=termination_signal(),
                 completion_basis='SIGTERM_after_verified_terminal_controlled_GO')
            break
        check(stage+'_after_native_read')
        if terminal.update(monotonic(), controlled_go_matches(group_id, states, group['native_ids'])):
            emit('stage_native_confirmed', stage=dict(stage=stage, color=controlled_go_state(group_id)),
                 native_confirmed_monotonic=terminal.confirmed_at, repeat_until_owner_teardown=True)
            emit('terminal_controlled_go_confirmed', native_states=states,
                 restore_required=True, natural_mode_restoration_certified=False)
        wait(.1)
    emit('experiment_complete', official_violation_certified=False,
         natural_mode_restoration_certified=False, restore_required=True)


def run(args):
    import rospy
    from std_msgs.msg import String
    from roslib.message import get_message_class
    from capture_traffic_light_phase_sequence import OfficialSender, ModernNativeReader, native_readback_issue

    group_id = parse_group(args.group_id)
    groups, geometry = load_geometry(args.geometry)
    group = next(row for row in groups if row['group_id'] == group_id)
    if not math.isfinite(args.max_seconds) or not 0 < args.max_seconds <= 600:
        raise ValueError('max_seconds must be finite within 0..600')
    if not math.isfinite(args.front_axle_offset_m) or not 0 <= args.front_axle_offset_m <= 6:
        raise ValueError('front axle offset must be finite within 0..6')
    output = args.output.expanduser().resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists():
        raise FileExistsError('Fresh artifacts output directory required')
    output.mkdir(parents=True)
    metadata = dict(development_only=True, experiment=EXPERIMENT, group=group, geometry=geometry,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        reused_helper_sha256={name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                              for name in ('control_traffic_light_policy_trial.py',
                                           'capture_traffic_light_phase_sequence.py',
                                           'observe_traffic_light_compliance.py')},
        max_seconds=args.max_seconds, front_axle_offset_m=args.front_axle_offset_m,
        departure_speed_mps=[MIN_DEPARTURE_SPEED_MPS, MAX_DEPARTURE_SPEED_MPS],
        min_front_distance_m=MIN_FRONT_DISTANCE_M, native_rpc_retries=2,
        native_per_call_timeout_sec=.15, fault_control_topic=FAULT_TOPIC,
        requested_controlled_go_state=controlled_go_state(group_id),
        yellow_durations_not_used_in_green_loss=getattr(args, 'yellow_durations_sec', None),
        stages=['fresh_inputs_allow_P', 'verified_R_stopped_D4_1s',
                'repeated_controlled_GO_SET_GET_until_actual_departure', 'drop_then_verified_R_3s',
                'fault_none_verified_R_2s', 'terminal_positive_GO_repeat_until_owner_SIGTERM'],
        restore_required=True,
        manual_natural_restore_required=True,
        cleanup_native_writes=False,
        limitations=['Controlled GO arming has NO fixed duration; it ends only at the departure condition or global deadline.',
            'A/B16, C48, D33 is an official controlled override, NOT a natural phase or a 15-second green program.',
            'Native GO GET is API state, not proof of pixel/model GT or exact phase at later SET.',
            'Native GO can change between reads; an explicit positive mismatching color never confirms GO.',
            'Natural-mode tl_color=-1/UNDEFINED cannot supply verified color and is never relabelled GREEN.',
            'No assumed yellow duration or remaining-phase time.',
            'Fault publish requests do not prove proxy application; compare actual proxy records.',
            'drop suppresses forwarded real perception outputs via the owner proxy; it does not stop raw camera/YOLO/CNN execution.',
            'Global deadline includes controlled GO SET/GET and actual-departure waiting.',
            'Exit2 means departure window unavailable, not compliance failure or success.',
            'The owner watchdog must handle every child exit and stop/park; this tool never commands a vehicle.',
            'This tool NEVER transmits a negative/native-default state, including preparation, terminal or cleanup.',
            'Only confirmed terminal controlled GO plus owner SIGTERM completes normally; other aborts remain nonzero.',
            'Cleanup performs no native signal writes; the last positive override may remain active.',
            'Manual MORAI restart/restoration is required before any natural-cycle full-course qualification.',
            'Sequence completion is not native official violation clearance or driving PASS.'])
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    writer = JsonlWriter(output/'events.jsonl')
    stop, lock, latest = threading.Event(), threading.RLock(), {}
    termination = [None]
    rospy.init_node('traffic_light_green_loss_phase_trial', anonymous=True, disable_signals=True)
    deadline = time.monotonic()+args.max_seconds
    def stop_on_signal(signum, _frame):
        termination[0] = signum
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_on_signal)

    def emit(event, **fields):
        writer.write(dict(event=event, monotonic=time.monotonic(), ros_ns=rospy.Time.now().to_nsec(), **fields))

    def callback(name):
        def receive(raw):
            mono, ros_ns = time.monotonic(), rospy.Time.now().to_nsec()
            with lock:
                if stop.is_set():
                    return
                try:
                    msgtype = raw._connection_header.get('type')
                    cls = get_message_class(msgtype)
                    if cls is None:
                        raise ValueError('Unavailable ROS message type')
                    message = cls().deserialize(raw._buff)
                    source = stamp_ns(getattr(getattr(message, 'header', None), 'stamp', None))
                    row = dict(receipt_monotonic=mono, receipt_ros_ns=ros_ns, source_stamp_ns=source)
                    if name == 'odom':
                        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
                        row.update(position_xyz=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                                   velocity_xyz=[v.x, v.y, v.z], frame=message.header.frame_id)
                    elif name == 'gear':
                        row['value'] = message.data
                    else:
                        row['control_message'] = primitive(message)
                    latest[name] = row
                    writer.write(dict(event='input_observation', input_name=name, message_type=msgtype, **row))
                except Exception as error:
                    latest.pop(name, None)
                    emit('input_observation_invalid', input_name=name, error=type(error).__name__+':'+str(error))
        return receive

    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, callback(name), queue_size=5)
                     for name, topic in (('odom', args.odometry_topic), ('gear', args.gear_topic), ('ctrl', '/ctrl_cmd'))]
    publisher = rospy.Publisher(FAULT_TOPIC, String, queue_size=1, latch=False)
    fault = FaultRepeater(lambda data: publisher.publish(String(data=data)), writer,
        fault_payload('none', group_id, EXPERIMENT, 'initializing'), lambda: rospy.Time.now().to_nsec())
    sender, reader, outcome, cleanup_errors = None, None, 'error', []
    last_positive_send_returned, last_verified_actual = {}, {}

    def snapshot():
        with lock:
            return dict(latest)

    def set_fault(mode, stage, immediate):
        fault.set(fault_payload(mode, group_id, EXPERIMENT, stage))
        if immediate:
            fault.emit()

    def positive_request(native_id, state, stage):
        requested_at = time.monotonic()
        emit('native_signal_request_attempt', stage=stage, native_id=native_id,
             requested_state=state, request_monotonic=requested_at)
        try:
            result = send_positive_signal(sender, native_id, state)
        except Exception as error:
            emit('native_signal_request_error', stage=stage, native_id=native_id,
                 requested_state=state, request_monotonic=requested_at,
                 error=type(error).__name__+':'+str(error))
            raise
        last_positive_send_returned[native_id] = dict(state=state, stage=stage,
            request_monotonic=requested_at, send_returned_monotonic=time.monotonic())
        emit('native_signal_requested', stage=stage, native_id=native_id,
             requested_state=state, send=result, request_monotonic=requested_at)

    def read_actual(stage, native_ids=None):
        states = {}
        for native_id in group['native_ids'] if native_ids is None else native_ids:
            requested_at = time.monotonic()
            try:
                result = reader(native_id, deadline=deadline)
                issue = native_readback_issue(result, native_id)
                actual = result['observations'][0]['tl_color'] if issue is None else None
                emit('native_signal_verified_actual', stage=stage, native_id=native_id,
                     request_monotonic=requested_at, response_monotonic=time.monotonic(),
                     actual_state=actual, verified=issue is None, validation_issue=issue, readback=result)
                if issue is not None:
                    raise TrialError('Native state/identity invalid: '+issue)
                states[native_id] = actual
                last_verified_actual[native_id] = dict(state=actual, stage=stage,
                    observed_monotonic=time.monotonic(), source='validated_native_GET',
                    current_at_cleanup_certified=False)
            except Exception as error:
                emit('native_read_error', stage=stage, native_id=native_id,
                     rpc_attempts=reader.last_rpc_attempts, error=type(error).__name__+':'+str(error))
                raise TrialError('Native observation failed; current color is unknown') from error
        return states

    def apply_red(stage):
        states = coordinated_native_cycle(group['native_ids'], 1,
            lambda native_id, state: positive_request(native_id, state, stage),
            lambda native_id: read_actual(stage, [native_id])[native_id])
        return all(value == 1 for value in states.values())

    def apply_go(stage):
        requested = controlled_go_state(group_id)
        return coordinated_native_cycle(group['native_ids'], requested,
            lambda native_id, state: positive_request(native_id, state, stage),
            lambda native_id: read_actual(stage, [native_id])[native_id])

    try:
        fault.start()
        wait_for_preparation_inputs(snapshot, time.monotonic, lambda: rospy.Time.now().to_nsec(),
            lambda: stop.is_set() or rospy.is_shutdown(), stop.wait, deadline, emit,
            publisher_error=lambda: fault.error)
        sender, reader = OfficialSender(), ModernNativeReader(timeout_sec=.15, rpc_retries=2)
        execute_sequence(group, snapshot, time.monotonic, lambda: rospy.Time.now().to_nsec(),
            stop.wait, lambda: stop.is_set() or rospy.is_shutdown(), deadline,
            set_fault, apply_red, apply_go, emit,
            publisher_error=lambda: fault.error, front_axle_offset_m=args.front_axle_offset_m,
            termination_signal=lambda: termination[0])
        outcome = 'sequence_completed'
    except TrialNotApplicable as error:
        outcome = 'not_applicable'
        emit('experiment_not_applicable', reason=str(error))
    except BaseException as error:
        emit('experiment_error', error=type(error).__name__+':'+str(error))
        raise
    finally:
        stop.set()
        try:
            fault.close(fault_payload('none', group_id, EXPERIMENT, 'finished'))
        except Exception as error:
            cleanup_errors.append('fault reset: '+str(error))
        for name, resource in (('reader', reader), ('sender', sender)):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    cleanup_errors.append(name+':'+str(error))
        for subscription in subscriptions:
            try:
                subscription.unregister()
            except Exception as error:
                cleanup_errors.append('unsubscribe: '+str(error))
        emit('cleanup_complete', outcome=outcome, cleanup_errors=cleanup_errors,
             native_writes_in_cleanup=0, restore_required=True,
             manual_natural_restore_required=True,
             last_positive_send_returned=last_positive_send_returned,
             last_verified_actual_observations=last_verified_actual,
             current_native_state_certified=False)
        (output/'summary.json').write_text(json.dumps(dict(outcome=outcome, group_id=group_id,
            experiment=EXPERIMENT, cleanup_errors=cleanup_errors,
            native_writes_in_cleanup=0, restore_required=True,
            manual_natural_restore_required=True,
            last_positive_send_returned=last_positive_send_returned,
            last_verified_actual_observations=last_verified_actual,
            current_native_state_certified=False,
            official_violation_certified=False, natural_mode_restoration_certified=False), indent=2)+'\n')
        writer.close()
        if cleanup_errors:
            raise TrialError('Cleanup failed: '+';'.join(cleanup_errors))
    return 2 if outcome == 'not_applicable' else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--group-id', required=True)
    parser.add_argument('--experiment', choices=(EXPERIMENT,), default=EXPERIMENT)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=180.)
    parser.add_argument('--front-axle-offset-m', type=float, default=3.)
    parser.add_argument('--odometry-topic', default='/localization/kinematic_state')
    parser.add_argument('--gear-topic', default='/vehicle/status/gear')
    parser.add_argument('--yellow-durations-sec', type=parse_yellow_durations,
                        help='Accepted for wrapper parity; green-loss does not use yellow-duration stages')
    raise SystemExit(run(parser.parse_args()))


if __name__ == '__main__':
    main()
