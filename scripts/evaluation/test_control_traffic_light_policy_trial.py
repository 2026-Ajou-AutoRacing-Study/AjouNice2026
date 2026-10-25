"""Pure phase/gate tests; no ROS, native RPC, UDP or vehicle command execution."""
import ast
import json
import math
from pathlib import Path
import signal

import pytest

import control_traffic_light_policy_trial as trial


GROUP = dict(group_id=201326, native_ids=['C1256W000052'], stop_line_xy=[[10, -2], [10, 2]])


def inputs(now=10., ros_ns=10000000000, gear=4, x=6., y=0., speed=0., yaw=0.):
    latest = {name: dict(receipt_monotonic=now, source_stamp_ns=None) for name in ('odom', 'gear', 'ctrl')}
    latest['odom'].update(position_xyz=[x, y, 0], quaternion_xyzw=[0, 0, math.sin(yaw/2), math.cos(yaw/2)],
                          velocity_xyz=[speed, 0, 0], frame='map', source_stamp_ns=ros_ns)
    latest['gear']['value'] = gear
    latest['ctrl']['control_message'] = dict(brake=1.)
    return latest


def test_only_four_explicit_mission_groups_can_receive_override_programs():
    assert trial.parse_group('A') == trial.parse_group('201351') == 201351
    assert trial.parse_group('B') == trial.parse_group('201365') == 201365
    assert trial.parse_group('C') == trial.parse_group('201290') == 201290
    assert trial.parse_group('D') == trial.parse_group('D-2') == trial.parse_group('201326') == 201326
    for value in ('D-1', '201397', '0', '201352', 'bad'):
        with pytest.raises(ValueError): trial.parse_group(value)


def test_stopped_yellow_is_nonnatural_1_3_5_second_stress_not_phase_estimation():
    for group, yellow, go in ((201351,4,16), (201365,4,16), (201290,4,48), (201326,5,33)):
        stages = trial.experiment_stages(group, 'stopped-yellow')
        assert [stage['seconds'] for stage in stages] == [1., 2., 3., 2., 5., 2., None]
        assert [stage['color'] for stage in stages] == [yellow, 1, yellow, 1, yellow, 1, go]
        assert all(stage['fault'] == 'none' for stage in stages)
        assert stages[-1]['action'] == 'controlled_go_until_owner_teardown'


def test_missing_signal_faults_do_not_change_native_red_GT():
    stages = trial.experiment_stages(201326, 'missing-signal')
    assert [stage['fault'] for stage in stages] == ['drop', 'wrong_id', 'none', 'none']
    assert [stage['color'] for stage in stages] == [1, 1, 1, 33]
    assert [stage['seconds'] for stage in stages] == [3., 3., 2., None]


def test_terminal_is_explicit_controlled_positive_GO_not_natural_mode_restore():
    for group in trial.GROUP_IDS:
        for experiment in ('stopped-yellow', 'missing-signal'):
            stages = trial.experiment_stages(group, experiment)
            assert stages[-1]['stage'] == 'terminal_controlled_go'
            assert stages[-1]['action'] == 'controlled_go_until_owner_teardown'
            assert stages[-1]['color'] == trial.controlled_go_state(group)
            assert stages[-1]['seconds'] is None
            assert all(stage['color'] in (1, 4, 5) for stage in stages[:-1])
            assert all('action' not in stage for stage in stages[:-1])


def test_start_requires_D4_not_P_fresh_input_and_stopped_speed():
    assert trial.start_gate(inputs(), GROUP, 10., 10000000000)['ready']
    assert trial.start_gate(inputs(gear=1), GROUP, 10., 10000000000)['reason'] == 'gear_not_D4'
    assert not trial.start_gate(inputs(speed=.1), GROUP, 10., 10000000000)['ready']
    assert not trial.start_gate(inputs(speed=.2), GROUP, 10., 10000000000)['ready']
    latest = inputs()
    latest['ctrl']['receipt_monotonic'] = 9.
    assert not trial.start_gate(latest, GROUP, 10., 10000000000)['ready']
    assert not trial.start_gate({}, GROUP, 10., 10000000000)['ready']


def test_start_front_point_is_before_actual_finite_segment_and_not_wrong_direction():
    gate = trial.start_gate(inputs(), GROUP, 10., 10000000000)
    assert gate['matches'][0]['front_point_map_xy_m'] == [9., 0.]
    assert gate['matches'][0]['forward_distance_m'] == 1.
    assert not trial.start_gate(inputs(x=8.), GROUP, 10., 10000000000)['ready']  # already beyond line
    assert not trial.start_gate(inputs(y=4.), GROUP, 10., 10000000000)['ready']  # outside segment
    assert not trial.start_gate(inputs(yaw=math.pi), GROUP, 10., 10000000000)['ready']
    assert not trial.start_gate(inputs(yaw=math.pi/2), GROUP, 10., 10000000000)['ready']
    assert not trial.start_gate(inputs(x=4.), GROUP, 10., 10000000000)['ready']  # too far


def test_map_odometry_is_not_given_raw_Ego_heading_correction():
    gate = trial.start_gate(inputs(), GROUP, 10., 10000000000)
    assert gate['matches'][0]['front_point_map_xy_m'][1] == 0.
    latest = inputs()
    latest['odom']['frame'] = 'morai_map'
    assert not trial.start_gate(latest, GROUP, 10., 10000000000)['ready']


def test_freshness_rejects_stale_or_future_odom_source_and_missing_messages():
    for source in (None, 0, 9000000000, 10000000001):
        latest = inputs()
        latest['odom']['source_stamp_ns'] = source
        assert not trial.fresh_inputs(latest, 10., 10000000000)[0]
    for name in ('odom', 'gear', 'ctrl'):
        latest = inputs()
        del latest[name]
        assert not trial.fresh_inputs(latest, 10., 10000000000)[0]
    # Header-less MORAI CtrlCmd uses explicitly recorded receipt freshness.
    assert trial.fresh_inputs(inputs(), 10., 10000000000)[0]


def test_initial_zero_quaternion_or_nonfinite_odom_never_arms_trial():
    latest = inputs()
    latest['odom']['quaternion_xyzw'] = [0., 0., 0., 0.]
    assert not trial.start_gate(latest, GROUP, 10., 10000000000)['ready']
    latest = inputs()
    latest['odom']['position_xyz'][0] = float('nan')
    assert not trial.start_gate(latest, GROUP, 10., 10000000000)['ready']


def test_preparation_waits_for_raw_fresh_inputs_allows_P_without_starting_phase():
    clock, rows, reads = [10.], [], [0]
    def snapshot():
        reads[0] += 1
        return {} if reads[0] < 3 else inputs(now=clock[0], ros_ns=int(clock[0]*1e9), gear=1)
    trial.wait_for_preparation_inputs(
        snapshot, lambda: clock[0], lambda: int(clock[0]*1e9), lambda: False,
        lambda seconds: clock.__setitem__(0, clock[0]+seconds), 11.,
        lambda event, **fields: rows.append(dict(event=event, **fields)))
    assert reads[0] == 3 and [row['ready'] for row in rows] == [False, False, True]
    parked = inputs(now=clock[0], ros_ns=int(clock[0]*1e9), gear=1)
    assert trial.fresh_inputs(parked, clock[0], int(clock[0]*1e9))[0]
    assert not trial.start_gate(parked, GROUP, clock[0], int(clock[0]*1e9))['ready']


def test_preparation_wait_is_bounded_and_stops_before_any_native_implementation():
    clock = [10.]
    with pytest.raises(trial.TrialError, match='preparation global deadline'):
        trial.wait_for_preparation_inputs(
            lambda: {}, lambda: clock[0], lambda: int(clock[0]*1e9), lambda: False,
            lambda seconds: clock.__setitem__(0, clock[0]+seconds), 10.25, lambda *a, **kw: None)
    with pytest.raises(trial.TrialError, match='Interrupted'):
        trial.wait_for_preparation_inputs(lambda: {}, lambda: 10., lambda: 10000000000,
            lambda: True, lambda seconds: None, 11., lambda *a, **kw: None)


def test_start_hold_must_be_one_continuous_ready_second():
    hold = trial.StartHold()
    assert not hold.update(True, 10.)
    assert not hold.update(True, 10.99)
    assert not hold.update(False, 11.)
    assert not hold.update(True, 12.)
    assert not hold.update(True, 12.99)
    assert hold.update(True, 13.)
    assert not hold.update(True, 1.)  # clock reversal discards old hold


def test_phase_duration_begins_with_actual_native_confirmation_not_request():
    phase = trial.PhaseClock(dict(seconds=1.), 10.)
    assert not phase.update(10.5, False)
    assert not phase.update(10.7, True)
    assert phase.confirmed_at == 10.7
    assert not phase.update(11., True)
    assert phase.update(11.701, True)


def test_native_apply_timeout_post_confirmation_mismatch_and_clock_reversal_raise():
    with pytest.raises(trial.TrialError): trial.PhaseClock(dict(seconds=1.), 10.).update(12., False)
    phase = trial.PhaseClock(dict(seconds=1.), 10.)
    phase.update(10.1, True)
    with pytest.raises(trial.TrialError): phase.update(10.2, False)
    with pytest.raises(trial.TrialError): phase.update(9., True)


def test_positive_only_sender_blocks_negative_bool_or_invalid_before_wire():
    sent = []
    sender = lambda native_id, state: sent.append((native_id, state)) or {'sent': True}
    for invalid in (-1, -2, True, False, 0, 32, 999, None, '1', 1.0):
        with pytest.raises(ValueError): trial.send_positive_signal(sender, 'D', invalid)
    assert sent == []
    for valid in (1, 4, 5, 16, 33, 48):
        assert trial.send_positive_signal(sender, 'D', valid)['sent']
    assert [state for _, state in sent] == [1, 4, 5, 16, 33, 48]


def test_terminal_positive_hold_confirms_once_then_requires_owner_SIGTERM():
    hold = trial.TerminalPositiveHold(33, 10.)
    assert not hold.update(10.1, False)
    assert hold.update(10.2, True)
    assert not hold.update(10.3, True)
    assert hold.confirmed_at == 10.2
    assert hold.finish_on_signal(signal.SIGTERM)
    with pytest.raises(trial.TrialError): hold.finish_on_signal(signal.SIGINT)
    with pytest.raises(trial.TrialError): trial.TerminalPositiveHold(16, 10.).finish_on_signal(signal.SIGTERM)
    assert trial.TerminalPositiveHold(48, 10.).update(10.1, True)


def test_custom_two_three_second_yellows_are_bounded_and_default_is_unchanged():
    assert trial.parse_yellow_durations('2,3') == (2., 3.)
    assert trial.parse_yellow_durations(' .5, 10 ') == (.5, 10.)
    for group in trial.GROUP_IDS:
        stages = trial.experiment_stages(group, 'stopped-yellow', (2., 3.))
        assert [stage['seconds'] for stage in stages] == [2., 2., 3., 2., None]
        assert stages[0]['stage'] == 'yellow_2s' and stages[2]['stage'] == 'yellow_3s'
        assert [stage['color'] for stage in stages] == [5 if group==201326 else 4, 1,
            5 if group==201326 else 4, 1, trial.controlled_go_state(group)]
    assert [stage['stage'] for stage in trial.experiment_stages(201351, 'stopped-yellow')] == [
        'yellow_1s','red_after_1s','yellow_3s','red_after_3s','yellow_5s','red_after_5s','terminal_controlled_go']
    for bad in ('', '0', '-1', '10.1', 'nan', 'inf', '2,,3', ','.join(['1']*13)):
        with pytest.raises(Exception): trial.parse_yellow_durations(bad)
    for bad in ((True,), (), (0.,), (math.nan,), [11.]):
        with pytest.raises(ValueError): trial.experiment_stages(201290, 'stopped-yellow', bad)


def test_C_both_heads_are_SET_before_any_GET_and_observations_are_not_fabricated():
    heads = list(trial.GROUP_IDS[201290])
    assert len(heads) == 2
    observed = {heads[0]:48, heads[1]:1}
    events = []
    def send(native_id, color): events.append(('SET',native_id,color))
    def read(native_id):
        events.append(('GET',native_id))
        return observed[native_id]
    states = trial.coordinated_native_cycle(heads, 48, send, read)
    assert [row[0] for row in events] == ['SET','SET','GET','GET']
    assert [row[1] for row in events[:2]] == heads
    assert states == observed and not all(state==48 for state in states.values())
    events.clear()
    with pytest.raises(ValueError): trial.coordinated_native_cycle(heads, -1, send, read)
    assert not events
    with pytest.raises(ValueError): trial.coordinated_native_cycle([heads[0],heads[0]], 48, send, read)


def test_terminal_positive_hold_does_not_mask_mismatch_timeout_or_clock_reversal():
    hold = trial.TerminalPositiveHold(33, 10.)
    with pytest.raises(trial.TrialError): hold.update(12., False)
    hold.update(10.1, True)
    with pytest.raises(trial.TrialError): hold.update(10.2, False)
    with pytest.raises(trial.TrialError): hold.update(9., True)
    for invalid in (1, 4, 5, -1, True):
        with pytest.raises(ValueError): trial.TerminalPositiveHold(invalid, 10.)


def test_fault_control_payload_has_no_color_GT_or_planner_signal_array():
    payload = trial.fault_payload('wrong_id', 201326, 'missing-signal', 'red_wrong_id')
    assert set(payload) == {'mode', 'group_id', 'experiment', 'stage'}
    assert payload['mode'] == 'wrong_id'
    with pytest.raises(ValueError): trial.fault_payload('green', 201326, 'bad', 'bad')


def test_fault_repeater_emits_only_explicit_fault_payload_and_final_none():
    sent, rows = [], []
    writer = type('Writer', (), {'write': lambda self, row: rows.append(row)})()
    repeater = trial.FaultRepeater(lambda text: sent.append(json.loads(text)), writer,
                                 trial.fault_payload('drop', 201326, 'missing-signal', 'drop'), lambda: 10)
    repeater.emit()
    repeater.close(trial.fault_payload('none', 201326, 'missing-signal', 'finished'))
    assert [row['mode'] for row in sent] == ['drop', 'none']
    assert all(row['event'] == 'fault_control_published' for row in rows)


def test_source_has_no_vehicle_publisher_service_or_stack_start_call():
    tree = ast.parse(Path(trial.__file__).read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    publishers = [node for node in calls if isinstance(node.func, ast.Attribute) and node.func.attr == 'Publisher']
    assert len(publishers) == 1 and isinstance(publishers[0].args[0], ast.Name)
    assert publishers[0].args[0].id == 'FAULT_TOPIC'
    attributes = {node.func.attr for node in calls if isinstance(node.func, ast.Attribute)}
    assert not attributes.intersection({'ServiceProxy', 'Popen', 'launch', 'system', 'SetTrafficLightInfo'})
    # Finally never changes a native signal, including on error or interrupt.
    for node in ast.walk(tree):
        if isinstance(node, ast.Try):
            cleanup_names = {child.func.id for statement in node.finalbody for child in ast.walk(statement)
                             if isinstance(child, ast.Call) and isinstance(child.func, ast.Name)}
            assert not cleanup_names.intersection({'sender', 'reset', 'send_positive_signal', 'apply_and_read'})
    run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run')
    native_try = next(node for node in run.body if isinstance(node, ast.Try))
    # Waiting comes before constructing either native transport. First override is R1.
    calls_in_order = [node.func.id for statement in native_try.body for node in ast.walk(statement)
                      if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)]
    assert calls_in_order.index('wait_for_preparation_inputs') < calls_in_order.index('OfficialSender')
    assert calls_in_order.index('wait_for_preparation_inputs') < calls_in_order.index('ModernNativeReader')
    assert calls_in_order.index('wait_for_preparation_inputs') < calls_in_order.index('apply_and_read')
    assert 'reset' not in calls_in_order
    first_apply = next(child for statement in native_try.body for child in ast.walk(statement)
                       if isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
                       and child.func.id == 'apply_and_read')
    # The explicitly selected real moving trial first holds its route's GO,
    # while the original stationary experiments still first override R1.
    assert isinstance(first_apply.args[0], ast.Call)
    assert isinstance(first_apply.args[0].func, ast.Name)
    assert first_apply.args[0].func.id == 'controlled_go_state'
    red_arming = next(node for node in ast.walk(native_try) if isinstance(node, ast.While)
                     and any(isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
                             and c.func.id == 'apply_and_read' and c.args
                             and isinstance(c.args[0], ast.Constant) and c.args[0].value == 1
                             for c in ast.walk(node)))
    assert red_arming is not None
