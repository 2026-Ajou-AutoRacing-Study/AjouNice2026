"""Synthetic CLOSED trial evidence only; no current/live artifacts are read."""
import ast
import json
from pathlib import Path
import sys

import pytest

import summarize_traffic_light_policy_trial as summary


def phase(stage, event, when, **extra):
    return dict(event=event, stage=stage, monotonic=when, **extra)


def odom(when, x=0., speed=0.):
    return dict(event='input_observation', input_name='odom', receipt_monotonic=when,
                position_xyz=[x, 0., 0.], velocity_xyz=[speed, 0., 0.])


def validator(success=True, parked=True):
    return dict(result=dict(status='PASS' if success else 'ERROR', reason='SUCCESS' if success else 'SAFETY_STOP_UNCONFIRMED',
                            drive_reason='SUCCESS'), safety=dict(shutdown=dict(
                                status='VERIFIED' if parked else 'UNCONFIRMED',
                                stop=dict(park_verified=parked, gear=1 if parked else 4))),
                events=[dict(event='CASE_RESULT', reason='SUCCESS')])


def test_completed_yellow_survives_partial_old_forced_green_and_goal_teardown():
    yellow = dict(stage='yellow_1s', color=5, seconds=1., fault='none')
    green = dict(stage='permitted_green', color=33, seconds=15., fault='none')
    rows = [phase(yellow, 'stage_requested', 0.),
            phase(yellow, 'stage_native_confirmed', .1, native_confirmed_monotonic=.1),
            *[odom(value) for value in (.1, .3, .5, .7, .9, 1.1)],
            phase(yellow, 'stage_complete', 1.11), phase(green, 'stage_requested', 1.2),
            phase(green, 'stage_native_confirmed', 1.3, native_confirmed_monotonic=1.3),
            dict(event='experiment_error', monotonic=2.1, error='Native phase changed after confirmation'),
            dict(event='cleanup_complete', monotonic=2.2)]
    report = summary.summarize(rows, [dict(event='validator_safety_teardown_started', monotonic=1.9)], [], {},
                               metadata=dict(stages=[yellow, green], group=dict(group_id=201326)), validator=validator())
    assert report['stages'][0]['execution_status'] == 'completed'
    assert report['stages'][1]['execution_status'] == 'partial'
    assert report['stages'][1]['observed_interval_end_monotonic'] == 1.9
    assert report['stationary_stress_stages_complete']
    assert not report['full_phase_program_complete']
    assert report['safety']['goal_success_observed'] and report['safety']['stop_P_verified']
    assert not report['automatic_overall_pass_issued']
    assert report['controlled_phase_errors']  # Experiment artifact, not counted as a road violation.
    assert not report['crossings']['official_violation_certified']


def test_absent_stages_are_unexecuted_and_unconfirmed_request_is_not_completion():
    first = dict(stage='yellow_1s', color=4, seconds=1., fault='none')
    absent = dict(stage='yellow_3s', color=4, seconds=3., fault='none')
    report = summary.summarize([phase(first, 'stage_requested', 1.), phase(first, 'stage_complete', 3.)], [], [], {},
                               metadata=dict(stages=[first, absent]))
    assert report['stages'][0]['execution_status'] == 'requested_but_native_unconfirmed'
    assert report['stages'][1]['execution_status'] == 'unexecuted'
    assert not report['stages'][0]['executed_complete']


def test_observed_elapsed_time_without_completion_label_is_only_partial():
    stage = dict(stage='yellow_1s', color=4, seconds=1., fault='none')
    rows = [phase(stage, 'stage_requested', 1.),
            phase(stage, 'stage_native_confirmed', 1.1, native_confirmed_monotonic=1.1),
            dict(event='cleanup_complete', monotonic=10.)]
    report = summary.summarize(rows, [], [], {}, metadata=dict(stages=[stage]))
    assert report['stages'][0]['execution_status'] == 'partial'
    assert not report['stages'][0]['completion_label_present']


def test_natural_release_completes_attempt_but_does_not_certify_color_or_mode():
    release = dict(stage='natural_release', color=None, seconds=0., action='reset_to_natural', fault='none')
    rows = [phase(release, 'stage_requested', 1.), phase(release, 'natural_release_attempted', 1.01),
            phase(release, 'stage_complete', 1.02)]
    report = summary.summarize(rows, [], [], {}, metadata=dict(stages=[release]))
    stage = report['stages'][0]
    assert stage['execution_status'] == 'natural_release_attempted_not_mode_certified'
    assert stage['native_confirmed_start_monotonic'] is None
    assert stage['natural_mode_restoration_certified'] is False


def test_old_natural_release_does_not_invent_a_missing_reset_attempt():
    release = dict(stage='natural_release', color=None, seconds=0., action='reset_to_natural', fault='none')
    rows = [phase(release, 'stage_requested', 1.), phase(release, 'stage_complete', 1.1)]
    stage = summary.summarize(rows, [], [], {}, metadata=dict(stages=[release]))['stages'][0]
    assert not stage['executed_complete']
    assert stage['natural_mode_restoration_certified'] is False


def terminal_stage():
    return dict(stage='terminal_controlled_go', color=33, seconds=None,
                action='controlled_go_until_owner_teardown', fault='none')


def test_terminal_GO_completes_at_confirmed_owner_SIGTERM_not_fixed_duration():
    terminal = terminal_stage()
    rows = [phase(terminal, 'stage_requested', 1.),
            phase(terminal, 'stage_native_confirmed', 1.2, native_confirmed_monotonic=1.2),
            phase(terminal, 'stage_complete', 4.1, native_confirmed_monotonic=1.2,
                  completion_basis='SIGTERM_after_verified_terminal_controlled_GO')]
    wrapper = [dict(event='validator_safety_teardown_started', monotonic=4.)]
    report = summary.summarize(rows, wrapper, [], {}, metadata=dict(stages=[terminal]),
        phase_summary=dict(manual_natural_restore_required=True, native_SET_in_cleanup=False,
                           last_requested_positive={'D': dict(state=33)}))
    stage = report['stages'][0]
    assert stage['execution_status'] == 'controlled_GO_completed_at_owner_teardown'
    assert stage['executed_complete'] and stage['terminal_positive_GO_confirmed']
    assert stage['native_confirmed_start_monotonic'] == 1.2
    assert stage['observed_interval_end_monotonic'] == 4.  # Exclude later child/stop-P cleanup.
    assert stage['native_or_action_complete_monotonic'] == 4.1
    assert stage['owner_teardown_context_verified']
    assert stage['natural_mode_restoration_certified'] is False
    assert report['full_phase_program_complete']
    assert report['native_cleanup_diagnostics']['manual_natural_restore_required'] is True
    assert report['native_cleanup_diagnostics']['native_SET_in_cleanup'] is False
    assert not report['native_cleanup_diagnostics']['current_native_state_certified']


def test_terminal_GO_without_end_label_is_partial_even_after_normal_goal():
    terminal = terminal_stage()
    rows = [phase(terminal, 'stage_requested', 1.),
            phase(terminal, 'stage_native_confirmed', 1.2, native_confirmed_monotonic=1.2),
            dict(event='cleanup_complete', monotonic=20.)]
    report = summary.summarize(rows, [dict(event='validator_safety_teardown_started', monotonic=4.)], [], {},
                               metadata=dict(stages=[terminal]), validator=validator())
    stage = report['stages'][0]
    assert stage['execution_status'] == 'partial'
    assert stage['observed_interval_end_monotonic'] == 4.
    assert not stage['executed_complete'] and report['safety']['goal_success_observed']


def test_terminal_go_needs_correct_completion_basis_and_native_confirmation():
    terminal = terminal_stage()
    for confirmation, basis in ((False, 'SIGTERM_after_verified_terminal_controlled_GO'),
                                (True, 'SIGINT'), (True, None)):
        rows = [phase(terminal, 'stage_requested', 1.)]
        if confirmation:
            rows.append(phase(terminal, 'stage_native_confirmed', 1.2, native_confirmed_monotonic=1.2))
        rows.append(phase(terminal, 'stage_complete', 4., completion_basis=basis))
        stage = summary.summarize(rows, [], [], {}, metadata=dict(stages=[terminal]))['stages'][0]
        assert not stage['executed_complete']


def test_terminal_confirmation_after_owner_teardown_is_not_valid_chronology():
    terminal = terminal_stage()
    rows = [phase(terminal, 'stage_requested', 1.),
            phase(terminal, 'stage_native_confirmed', 4.1, native_confirmed_monotonic=4.1),
            phase(terminal, 'stage_complete', 4.2, completion_basis='SIGTERM_after_verified_terminal_controlled_GO')]
    report = summary.summarize(rows, [dict(event='validator_safety_teardown_started', monotonic=4.)], [], {},
                               metadata=dict(stages=[terminal]))
    assert report['stages'][0]['owner_teardown_precedes_native_confirmation']
    assert not report['stages'][0]['executed_complete']
    assert report['stages'][0]['observed_interval_end_monotonic'] is None


def test_explicit_confirmed_clock_in_greenloss_complete_is_not_inferred_from_duration():
    stage = dict(stage='red_restored', color=1, seconds=2., fault='none')
    rows = [phase(stage, 'stage_requested', 1.),
            phase(dict(stage='red_restored'), 'stage_complete', 3.2, native_confirmed_monotonic=1.1)]
    report = summary.summarize(rows, [], [], {}, metadata=dict(stages=[stage]))
    assert report['stages'][0]['executed_complete']
    assert report['stages'][0]['native_confirmed_start_monotonic'] == 1.1
    assert report['stages'][0]['native_confirmation_start_evidence'] == 'stage_complete.native_confirmed_monotonic'


def test_descriptive_greenloss_metadata_is_safe_and_cleanup_alias_preserved():
    terminal = terminal_stage()
    rows = [phase(terminal, 'stage_requested', 1.),
            phase(terminal, 'stage_complete', 4., native_confirmed_monotonic=1.2,
                  completion_basis='SIGTERM_after_verified_terminal_controlled_GO')]
    report = summary.summarize(rows, [], [], {}, metadata=dict(stages=['some descriptive preparation text']),
                               phase_summary=dict(native_writes_in_cleanup=0, manual_natural_restore_required=True))
    assert report['stages'][0]['executed_complete']
    assert not report['full_phase_plan_enumeration_verified']
    assert not report['full_phase_program_complete']  # Cannot enumerate missing descriptive phases.
    assert report['native_cleanup_diagnostics']['native_SET_in_cleanup'] is False


def test_preserved_JPEG_pair_never_changes_old_unknown_native_permission():
    images = dict(status='complete_original_pair', before=dict(path='before.jpg'), after=dict(path='after.jpg'),
                  native_GT_replaced=False, independent_pixel_review_required=True)
    document = dict(crossings=[dict(group_id=201326, crossing_jpeg_bracket=images,
        native_bracket_evaluation={'0.25': dict(verified=False, route_permission_proxy=None),
                                   '0.5': dict(verified=False, route_permission_proxy=None)})])
    result = summary.summarize_crossings(document)
    assert result['events'][0]['crossing_jpeg_bracket'] == images
    assert result['counts_by_native_age_bound_sec']['0.25'] == {'unverified': 1}


def test_motion_metrics_keep_speed_displacement_thresholds_separate():
    samples = [dict(monotonic=value, position=[x, 0, 0], speed=speed)
               for value, x, speed in [(0., 0., 0.), (.25, .1, .31), (.5, .3, 0.)]]
    result = summary.summarize_motion(samples, 0., .5)
    assert result['speed_mps']['max'] == .31
    assert result['maximum_XY_displacement_from_first_sample_m'] == .3
    assert result['departure_proxy_both'] and result['coverage_verified']
    boundary = summary.summarize_motion([dict(monotonic=0., position=[0, 0, 0], speed=.3),
        dict(monotonic=.5, position=[.25, 0, 0], speed=.3)], 0., .5)
    assert not boundary['departure_proxy_any']
    assert not summary.summarize_motion(samples[:1], 0., .5)['coverage_verified']
    assert not summary.summarize_motion([], 0., 1.)['coverage_verified']


def test_raw_Ego_and_localization_remain_separate_no_duplicate_odom_count():
    observer = [dict(event='ROS_observation', topic='/Ego_topic', receipt_monotonic=1.,
                message=dict(position=dict(x=100., y=200., z=0.), velocity=dict(x=0., y=0., z=0.))),
                dict(event='ROS_observation', topic='/localization/kinematic_state', receipt_monotonic=1.,
                message=dict(pose=dict(pose=dict(position=dict(x=1., y=2., z=0.))),
                             twist=dict(twist=dict(linear=dict(x=.4, y=0., z=0.)))))]
    local, ego, source = summary.motion_samples(observer, [odom(1., speed=5.)])
    assert len(local) == len(ego) == 1
    assert local[0]['speed'] == .4 and ego[0]['speed'] == 0.
    assert source == 'observer/localization'


def test_drop_and_wrong_id_need_actual_proxy_effect_not_only_requested_command():
    control = [dict(event='fault_control_published', receipt_monotonic=.1,
                    payload=dict(mode='wrong_id'))]
    wrapper = [dict(event='signal_proxy', monotonic=.2, mode='wrong_id', input_ids=[201326], output_ids=[0])]
    result = summary.fault_coverage(wrapper, control, 0., 1., 'wrong_id', 201326)
    assert result['coverage_verified'] and result['effect_confirmed_on_target_frames'] == 1
    wrapper[0]['output_ids'] = [201326]
    assert not summary.fault_coverage(wrapper, control, 0., 1., 'wrong_id', 201326)['coverage_verified']
    wrapper[0].update(mode='drop', output_ids=None)
    assert summary.fault_coverage(wrapper, [], 0., 1., 'drop', 201326)['coverage_verified']
    assert not summary.fault_coverage([], control, 0., 1., 'drop', 201326)['coverage_verified']


def test_native_crossing_unknown_deny_and_permission_are_different():
    document = dict(crossings=[dict(group_id=201326, native_bracket_evaluation={
        '0.25': dict(verified=False, route_permission_proxy=None),
        '0.5': dict(verified=True, color=1, route_permission_proxy=False, native_heads=[dict(before_color=1, after_color=1)])}),
        dict(group_id=201326, native_bracket_evaluation={'0.25': dict(verified=True, color=33, route_permission_proxy=True),
                                                       '0.5': dict(verified=True, color=33, route_permission_proxy=True)})])
    report = summary.summarize_crossings(document)
    assert report['counts_by_native_age_bound_sec']['0.25']['unverified'] == 1
    assert report['counts_by_native_age_bound_sec']['0.5']['verified_no_permission_proxy'] == 1
    assert report['events'][0]['native_bracket_evaluation']['0.5']['native_heads'][0]['before_color'] == 1
    assert report['official_violation_certified'] is False


def test_fresh_synthetic_wire_stamp_and_old_adopted_lamp_age_are_not_merged():
    topic = '/perception/traffic_light_recognition/traffic_signals'
    rows = [dict(event='ROS_observation', topic=topic, receipt_ros_ns=12000000000,
                 message=dict(stamp=dict(sec=11, nanosec=990000000))),
            dict(event='ROS_observation', topic=topic, receipt_ros_ns=12000000000,
                 message=dict(stamp=dict(sec=10, nanosec=0)))]
    for observed, held, adopted, age in ((False, False, None, None), (True, True, 10000000000, 1.9)):
        rows.append(dict(event='ROS_observation', topic='/traffic_light_node/debug_runtime', receipt_ros_ns=12000000000,
            message=dict(parsed_json=dict(frame_decision=dict(observation_timing=dict(output_is_observed=observed,
                output_is_held=held, adopted_observation_source_stamp_ns=adopted, output_observation_age_sec=age)),
                frame_timing=dict(callback_to_output_sec=.02, selection_including_map_wait_sec=.012,
                                  near_rescue_detector_and_association_sec=.009)))))
    report = summary.summarize_ages_and_runtime(rows, topic)
    assert report['standard_signal_stamp_age_at_observer_sec']['mean'] == pytest.approx(1.005)
    assert report['debug_adopted_source_age_at_observer_sec']['mean'] == 2.
    assert report['synthetic_fallback_frames'] == report['held_observation_frames'] == 1
    assert report['runtime_ms']['callback_to_output_sec']['p95'] == 20.


def test_goal_SUCCESS_with_unconfirmed_shutdown_is_not_safe_P_completion():
    safety = summary.summarize_safety([], validator(success=False, parked=False))
    assert safety['goal_success_observed']
    assert not safety['stop_P_verified'] and not safety['validator_final_success']
    assert safety['goal_SUCCESS_is_signal_policy_pass'] is False
    assert safety['separate_later_P_recovery_is_not_this_trial_completion']


def test_loader_reads_explicit_closed_fixture_and_preserves_truncated_record_warning(tmp_path):
    (tmp_path/'phase_controller').mkdir()
    (tmp_path/'observer').mkdir()
    (tmp_path/'phase_controller/events.jsonl').write_text('{"event":"experiment_error","error":"old control artifact"}\n{"broken"\n')
    (tmp_path/'wrapper_events.jsonl').write_text('[]\n')
    (tmp_path/'observer/observations.jsonl').write_text('')
    (tmp_path/'observer/crossings.json').write_text('{"crossings": []}')
    (tmp_path/'phase_controller/metadata.json').write_text('{"stages": []}')
    (tmp_path/'phase_controller/summary.json').write_text('{"experiment_completed": false}')
    (tmp_path/'selection.json').write_text('{}')
    before = (tmp_path/'phase_controller/events.jsonl').read_bytes()
    report, evidence, warnings = summary.load_closed_artifacts(tmp_path)
    assert not report['automatic_overall_pass_issued'] and len(evidence) == 7
    assert sum('JSONL' in warning for warning in warnings) == 2
    assert (tmp_path/'phase_controller/events.jsonl').read_bytes() == before


def test_summary_module_has_no_ROS_native_model_or_process_dependency():
    tree = ast.parse(Path(summary.__file__).read_text())
    imports = {node.name.split('.')[0] for node in ast.walk(tree) if isinstance(node, ast.alias)}
    assert not imports.intersection({'rospy', 'grpc', 'torch', 'ultralytics', 'subprocess', 'socket'})


def test_nonexistent_explicit_validator_path_is_an_input_error_not_unverified_stop(tmp_path):
    with pytest.raises(FileNotFoundError, match='validator execution'):
        summary.load_closed_artifacts(tmp_path, tmp_path/'wrong_case_path')


def test_CLI_rejects_missing_validator_before_any_summary_output(tmp_path, monkeypatch, capsys):
    output = tmp_path/'must_not_be_created.json'
    monkeypatch.setattr(sys, 'argv', ['summarize', str(tmp_path), '--validator-execution',
        str(tmp_path/'nonexistent_execution'), '--output', str(output), '--confirm-closed'])
    with pytest.raises(SystemExit) as error:
        summary.main()
    assert error.value.code == 2
    assert '--validator-execution must name an existing' in capsys.readouterr().err
    assert not output.exists()


def test_existing_partial_validator_dir_without_receipt_remains_unknown(tmp_path):
    wrapper = tmp_path/'wrapper'
    execution = tmp_path/'partial_case'
    wrapper.mkdir()
    execution.mkdir()
    report, _, warnings = summary.load_closed_artifacts(wrapper, execution)
    assert not report['safety']['stop_P_verified']
    assert report['safety']['safety_shutdown_status'] is None
    assert any('safety_receipt.json' in warning for warning in warnings)
