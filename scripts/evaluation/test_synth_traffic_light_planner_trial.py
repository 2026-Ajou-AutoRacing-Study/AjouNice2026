"""Pure manual-fixture state machines, never ROS/native/MORAI/control execution."""
import ast
from pathlib import Path
import signal
import types

import pytest

import synth_traffic_light_planner_trial as trial

GROUP = dict(group_id=201326, stop_line_xy=[[30., -5.], [30., 5.]])


def snapshot(now, distance, speed=0., group_id=201326, gear=4, y=0., yaw=0., source_ns=None):
    import math
    source_ns = int((now-.01)*1e9) if source_ns is None else source_ns
    common = dict(receipt_monotonic=now, source_stamp_ns=None)
    return dict(odom=dict(common, frame='map', source_stamp_ns=source_ns,
                    position_xyz=[30-3*math.cos(yaw)-distance, y-3*math.sin(yaw), 0.],
                    quaternion_xyzw=[0, 0, math.sin(yaw/2), math.cos(yaw/2)], velocity_xyz=[speed, 0., 0.]),
                gear=dict(common, value=gear), ctrl=dict(common, control_message=dict(longlCmdType=1, accel=.2, brake=0)),
                route=dict(common, value=group_id))


def step(machine, now, distance, speed=0., **kwargs):
    return machine.step(snapshot(now, distance, speed, group_id=machine.config.group_id, **kwargs), now, int(now*1e9))


def test_explicit_fixture_lamps_for_all_four_missions_are_not_native_GT():
    assert trial.phase_tokens(201351, 'GO') == ['GREEN_CIRCLE']
    assert trial.phase_tokens(201365, 'GO') == ['GREEN_CIRCLE']
    assert trial.phase_tokens(201290, 'GO') == ['GREEN_CIRCLE', 'GREEN_LEFT_ARROW']
    assert trial.phase_tokens(201326, 'GO') == ['RED_CIRCLE', 'GREEN_LEFT_ARROW']
    assert trial.phase_tokens(201326, 'YELLOW') == ['RED_CIRCLE', 'YELLOW_CIRCLE']
    for group in (201351, 201365, 201290): assert trial.phase_tokens(group, 'YELLOW') == ['YELLOW_CIRCLE']
    with pytest.raises(ValueError): trial.phase_tokens(0, 'GO')


def test_fixture_stamp_preserves_integer_time_without_perception_libexec_import():
    message = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
    trial.assign_fixture_stamp(message, 1792936808, 123456789)
    assert message.stamp.sec == 1792936808 and message.stamp.nanosec == 123456789
    with pytest.raises(ValueError): trial.assign_fixture_stamp(message, 10., 1)
    with pytest.raises(ValueError): trial.assign_fixture_stamp(message, 10, 1000000000)


def test_snapshot_clock_follows_callback_update_and_input_copy_under_one_lock():
    events = []
    clock = [10.210163466]  # The pre-lock main time from the reproduced race.
    latest = snapshot(clock[0], 1.)
    class CallbackLock:
        active = False
        def __enter__(self):
            self.active = True
            events.append('lock_enter')
            # The callback updated gear after old main 'now', before main lock.
            latest['gear'] = dict(latest['gear'], receipt_monotonic=10.210365813)
            clock[0] = 10.2105
            return self
        def __exit__(self, *_):
            events.append('lock_exit')
            self.active = False
    lock = CallbackLock()
    def monotonic():
        assert lock.active
        events.append('monotonic_after_snapshot')
        return clock[0]
    def ros_now():
        assert lock.active
        events.append('ROS_now_after_snapshot')
        return types.SimpleNamespace(to_nsec=lambda: int(clock[0]*1e9))
    captured, now, ros = trial.snapshot_with_current_clocks(lock, latest, monotonic, ros_now)
    assert trial.fresh_inputs(captured, now, ros.to_nsec())[0]
    assert events == ['lock_enter', 'monotonic_after_snapshot', 'ROS_now_after_snapshot', 'lock_exit']
    # Subsequent callbacks replace their row; they cannot make this snapshot future.
    latest['gear'] = dict(latest['gear'], receipt_monotonic=11.)
    assert captured['gear']['receipt_monotonic'] == 10.210365813
    assert trial.fresh_inputs(captured, now, ros.to_nsec())[0]


def test_main_uses_atomic_snapshot_clock_helper_and_keeps_strict_future_rejection():
    tree = ast.parse(Path(trial.__file__).read_text())
    run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run')
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
               and node.func.id == 'snapshot_with_current_clocks' for node in ast.walk(run))
    data = snapshot(10., 1.)
    data['gear']['receipt_monotonic'] = 10.000000001
    assert not trial.fresh_inputs(data, 10., 10000000000)[0]  # No tolerance relaxation.


def test_configuration_enforces_safe_stoppable_and_close_trigger_ranges():
    trial.FixtureConfig('moving-yellow-stoppable', 201326)
    trial.FixtureConfig('moving-yellow-close', 201326)
    with pytest.raises(ValueError): trial.FixtureConfig('moving-yellow-stoppable', 201326, trigger_min_m=5.)
    with pytest.raises(ValueError): trial.FixtureConfig('moving-yellow-stoppable', 201326, trigger_max_speed_mps=5.)
    with pytest.raises(ValueError): trial.FixtureConfig('moving-yellow-close', 201326, trigger_max_m=2.)
    with pytest.raises(ValueError): trial.FixtureConfig('loss-postfront', 201326, probe_seconds=3.)


def test_initial_moving_GO_requires_fresh_active_route_D4_stop8_25m():
    machine = trial.PlannerFixture(trial.FixtureConfig('moving-yellow-stoppable', 201326), GROUP)
    assert machine.step({}, 10., 10000000000)['publish_signal'] is False
    wrong_route = snapshot(10.1, 20.)
    wrong_route['route']['value'] = 0
    assert machine.step(wrong_route, 10.1, 10100000000)['status'] == 'waiting_route'
    parked = step(machine, 10.2, 20., gear=1)
    assert parked['manual_phase'] == 'RED' and not machine.armed
    moving = step(machine, 10.3, 20., speed=.2)
    assert moving['manual_phase'] == 'RED' and not machine.armed
    initial = step(machine, 10.4, 20.)
    assert initial['manual_phase'] == 'GO' and machine.armed
    assert initial['planner_only'] and not initial['official_lamp_compliance_certified']


def test_initial_window_cannot_start_at_stopline_or_wrong_finite_segment():
    machine = trial.PlannerFixture(trial.FixtureConfig('moving-yellow-close', 201326), GROUP)
    with pytest.raises(trial.FixtureNotApplicable): step(machine, 10., 1.)
    machine = trial.PlannerFixture(trial.FixtureConfig('moving-yellow-close', 201326), GROUP)
    with pytest.raises(trial.FixtureNotApplicable): step(machine, 10., 20., y=10.)


@pytest.mark.parametrize('group_id', [201351, 201365, 201290, 201326])
@pytest.mark.parametrize('duration', [2., 3.])
def test_stopped_yellow_matrix_holds_and_emits_complete_handoff_only_after_motion_coverage(group_id, duration):
    group = dict(GROUP, group_id=group_id)
    machine = trial.PlannerFixture(trial.FixtureConfig('stopped-yellow', group_id, probe_seconds=duration), group)
    assert step(machine, 10., 1.)['manual_phase'] == 'RED'
    assert step(machine, 10.9, 1.)['manual_phase'] == 'RED'
    yellow = step(machine, 11., 1.)
    assert yellow['manual_phase'] == 'YELLOW' and not yellow['handoff_real']
    for index in range(1, int(duration*10)):
        decision = step(machine, 11.+index*.1, 1.)
        assert decision['manual_phase'] == 'YELLOW' and not decision['handoff_real']
    complete = step(machine, 11.+duration, 1.)
    assert complete['stage'] == 'fixture_complete' and complete['handoff_real']
    assert complete['motion_stage_captured'] and not complete['publish_signal']
    assert not complete['behavior_PASS_certified']
    assert machine.finish_on_signal(signal.SIGTERM)
    # Keep heartbeat alive after handoff without inventing further signal input.
    heartbeat = machine.step({}, 12.+duration, int((12.+duration)*1e9))
    assert heartbeat['stage'] == 'fixture_complete' and heartbeat['handoff_real']


def test_stopped_hold_is_reset_and_never_begins_on_initial_P():
    machine = trial.PlannerFixture(trial.FixtureConfig('stopped-yellow', 201326), GROUP)
    step(machine, 10., 1., gear=1)
    step(machine, 11., 1.)
    step(machine, 11.5, 1., speed=.2)
    assert step(machine, 12., 1.)['manual_phase'] == 'RED'
    assert step(machine, 12.9, 1.)['manual_phase'] == 'RED'
    assert step(machine, 13., 1.)['manual_phase'] == 'YELLOW'


def test_stopped_hold_does_not_bridge_missing_inputs_or_P_or_route_changes():
    for interruption in ('input', 'P', 'route'):
        machine = trial.PlannerFixture(trial.FixtureConfig('stopped-yellow', 201326), GROUP)
        step(machine, 10., 1.)
        if interruption == 'input': machine.step({}, 10.5, 10500000000)
        elif interruption == 'P': step(machine, 10.5, 1., gear=1)
        else:
            changed = snapshot(10.5, 1.)
            changed['route']['value'] = 0
            machine.step(changed, 10.5, 10500000000)
        assert step(machine, 11., 1.)['manual_phase'] == 'RED'
        assert step(machine, 11.9, 1.)['manual_phase'] == 'RED'
        assert step(machine, 12., 1.)['manual_phase'] == 'YELLOW'


def test_stoppable_moving_Y_starts_only_inside_distance_and_speed_gate():
    machine = trial.PlannerFixture(trial.FixtureConfig('moving-yellow-stoppable', 201326), GROUP)
    step(machine, 10., 20.)
    assert step(machine, 10.1, 19.9, speed=.5)['manual_phase'] == 'GO'
    trigger = step(machine, 10.2, 19.8, speed=1.2)
    assert trigger['manual_phase'] == 'YELLOW'
    assert trigger['trigger']['projection']['forward_distance_m'] >= 10.
    assert 1 <= trigger['trigger']['speed_mps'] <= 3


def test_close_Y_does_not_force_input_when_window_missed():
    machine = trial.PlannerFixture(trial.FixtureConfig('moving-yellow-close', 201326), GROUP)
    step(machine, 10., 20.)
    step(machine, 10.1, 10., speed=4.)
    assert step(machine, 10.2, 2., speed=4.)['manual_phase'] == 'GO'
    with pytest.raises(trial.FixtureNotApplicable): step(machine, 10.3, .2, speed=4.)
    assert machine.started_at is None
    machine = trial.PlannerFixture(trial.FixtureConfig('moving-yellow-close', 201326), GROUP)
    step(machine, 20., 20.)
    step(machine, 20.1, 10., speed=4.)
    trigger = step(machine, 20.2, .7, speed=4.)
    assert trigger['manual_phase'] == 'YELLOW' and not trigger['handoff_real']


@pytest.mark.parametrize('mode,expected', [('loss-postfront', None), ('postfront-red', 'RED')])
def test_postfront_modes_require_actual_finite_front_crossing_and_preserve_signed_after_distance(mode, expected):
    machine = trial.PlannerFixture(trial.FixtureConfig(mode, 201326), GROUP)
    step(machine, 10., 20.)
    step(machine, 10.1, 10., speed=4.)
    assert step(machine, 10.2, .5, speed=4.)['manual_phase'] == 'GO'
    trigger = step(machine, 10.3, -.2, speed=4.)
    assert machine.crossed and trigger['manual_phase'] == expected
    assert trigger['trigger']['projection']['signed_distance_to_line_m'] < 0
    assert trigger['trigger']['projection']['forward_distance_m'] < 0
    assert not trigger['handoff_real']


def test_loss_prefront_is_real_nonpublication_not_empty_or_RED_signal():
    machine = trial.PlannerFixture(trial.FixtureConfig('loss-prefront', 201326), GROUP)
    step(machine, 10., 20.)
    loss = step(machine, 10.1, 11., speed=2.)
    assert loss['status'] == 'probe_running' and loss['manual_phase'] is None
    assert not loss['publish_signal'] and loss['lamp_tokens'] == []
    assert not loss['handoff_real']  # Root must not fill this gap with real YOLO.


def begin_RED_settling(mode):
    machine = trial.PlannerFixture(trial.FixtureConfig(mode, 201326), GROUP)
    step(machine, 10., 20.)
    distance = 19.9 if mode == 'moving-yellow-stoppable' else 11.
    step(machine, 10.1, distance, speed=1.2)
    for index in range(1, 23):
        now = 10.1+index*.1
        decision = step(machine, now, distance-index*.05, speed=.5)
        if decision['stage'] == 'await_stop_red':
            return machine, now, distance-index*.05, decision
    raise AssertionError('Probe did not reach RED settling')


@pytest.mark.parametrize('mode', ['moving-yellow-stoppable', 'loss-prefront'])
def test_prefront_probe_end_keeps_RED_until_continuous_distinct_one_second_stop(mode):
    machine, now, distance, settling = begin_RED_settling(mode)
    frozen = dict(settling['probe_motion_summary'])
    assert settling['manual_phase'] == 'RED' and not settling['handoff_real']
    assert settling['stage'] == 'await_stop_red' and not machine.complete
    # Still moving after the raw probe is over: no early handback of real G.
    moving = step(machine, now+.1, distance-.05, speed=2.)
    assert moving['stage'] == 'await_stop_red' and moving['manual_phase'] == 'RED'
    assert not moving['handoff_real']
    assert moving['probe_motion_summary'] == frozen  # RED settling is separate.
    first_stop = now+.2
    step(machine, first_stop, distance-.1, speed=0.)
    for index in range(1, 10):
        waiting = step(machine, first_stop+index*.1, distance-.1, speed=0.)
        assert waiting['manual_phase'] == 'RED' and not waiting['handoff_real']
    complete = step(machine, first_stop+1.01, distance-.1, speed=0.)
    assert complete['stage'] == 'fixture_complete' and complete['handoff_real']
    assert complete['red_settle_stop']['verified_prefront_stop']
    assert complete['red_settle_stop']['stopped_wall_span_sec'] >= 1.
    assert complete['red_settle_stop']['stopped_source_span_sec'] >= 1.
    assert complete['red_settle_stop']['distinct_stopped_source_samples'] >= 2
    assert complete['probe_motion_summary'] == frozen
    assert complete['red_settle_stop']['maximum_settle_speed_mps'] == 2.


@pytest.mark.parametrize('mode', ['moving-yellow-stoppable', 'loss-prefront'])
def test_crossing_before_settled_stop_is_error_and_never_qualifies_handoff(mode):
    machine, now, _, _ = begin_RED_settling(mode)
    step(machine, now+.1, .1, speed=.5)
    with pytest.raises(trial.FixtureError, match='crossed before verified prefront stop'):
        step(machine, now+.2, -.1, speed=.5)
    assert not machine.complete


def test_prefront_stop_hold_resets_if_vehicle_moves_again():
    machine, now, distance, _ = begin_RED_settling('loss-prefront')
    step(machine, now+.1, distance, speed=0.)
    for index in range(2, 9): step(machine, now+index*.1, distance, speed=0.)
    step(machine, now+.9, distance, speed=.2)
    step(machine, now+1., distance, speed=0.)
    for index in range(11, 20):
        assert not step(machine, now+index*.1, distance, speed=0.)['handoff_real']
    assert step(machine, now+2.01, distance, speed=0.)['handoff_real']


def test_RED_settling_has_bounded_twenty_second_limit():
    machine, now, distance, _ = begin_RED_settling('loss-prefront')
    with pytest.raises(trial.FixtureError, match='bounded20seconds'):
        # Pure helper call bypasses the separate ego-gap guard to isolate this limit.
        projection = trial.front_projection(snapshot(now+20.01, distance)['odom'], GROUP,
                                             segment_index=machine.segment_index, approach_sign=machine.approach_sign)
        machine._await_stopped_before_line(snapshot(now+20.01, distance), now+20.01, projection, .2)


def test_stale_inputs_or_invalid_motion_samples_never_authorize_handoff():
    machine = trial.PlannerFixture(trial.FixtureConfig('stopped-yellow', 201326), GROUP)
    step(machine, 10., 1.)
    step(machine, 11., 1.)
    with pytest.raises(trial.FixtureError): machine.step({}, 11.1, 11100000000)
    assert not machine.complete
    machine = trial.PlannerFixture(trial.FixtureConfig('stopped-yellow', 201326), GROUP)
    step(machine, 20., 1.)
    step(machine, 21., 1., source_ns=20990000000)
    with pytest.raises(trial.FixtureError): step(machine, 23., 1., source_ns=20990000000)
    assert not machine.complete


def test_owner_SIGTERM_before_probe_done_is_partial_not_success():
    machine = trial.PlannerFixture(trial.FixtureConfig('stopped-yellow', 201326), GROUP)
    step(machine, 10., 1.)
    with pytest.raises(trial.FixtureError): machine.finish_on_signal(signal.SIGTERM)
    with pytest.raises(trial.FixtureError): machine.finish_on_signal(signal.SIGINT)


def test_localization_is_not_raw_Ego_heading_corrected_again():
    projection = trial.front_projection(snapshot(10., 1.)['odom'], GROUP)
    assert projection['front_xy_m'] == [29., 0.]
    assert projection['forward_distance_m'] == 1.


def test_source_only_owns_fixed_evaluation_signal_and_status_topics_no_native_or_vehicle_calls():
    tree = ast.parse(Path(trial.__file__).read_text())
    imports = {item.name for item in ast.walk(tree) if isinstance(item, ast.alias)}
    assert not any('native' in name.lower() or 'grpc' in name.lower() or name in ('socket', 'subprocess') for name in imports)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    publishers = [node for node in calls if isinstance(node.func, ast.Attribute) and node.func.attr == 'Publisher']
    assert {node.args[0].id for node in publishers} == {'SIGNAL_TOPIC', 'STATUS_TOPIC'}
    attrs = {node.func.attr for node in calls if isinstance(node.func, ast.Attribute)}
    assert not attrs.intersection({'ServiceProxy', 'Popen', 'sendto', 'SetTrafficLightInfo'})
    assert trial.SIGNAL_TOPIC.startswith('/evaluation/') and trial.STATUS_TOPIC.startswith('/evaluation/')
