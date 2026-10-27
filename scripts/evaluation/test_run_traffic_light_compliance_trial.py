from types import SimpleNamespace
from run_traffic_light_compliance_trial import temporal_video_launch_arguments
from run_traffic_light_compliance_trial import owned_child_exit_grace_sec
from run_traffic_light_compliance_trial import validate_video_request

import pytest


def test_front_only_capture_does_not_require_or_capture_a_desktop_window():
    validate_video_request(True, True, 'unknown_grace', None)
    validate_video_request(True, False, 'current', '0x123')
    validate_video_request(False, False, None, None)
    for args in ((False, True, 'current', None), (True, False, 'current', None),
                 (True, True, None, None)):
        with pytest.raises(ValueError):
            validate_video_request(*args)


def test_camera_recorder_gets_time_to_finalize_its_owned_encoder():
    assert owned_child_exit_grace_sec('front_video') >= 28
    assert owned_child_exit_grace_sec('observer') == 8
    assert owned_child_exit_grace_sec('simulator_video') == 8


def test_unknown_grace_is_explicit_bounded_and_keeps_known_color_evidence():
    original = {'repo_root': '/repo'}
    planning = temporal_video_launch_arguments('ajounice_planning_integration',
        'planning.launch', original, unknown_grace_sec=.2)
    assert planning == dict(original, signal_compliance_unknown_grace_sec=.2)
    traffic = temporal_video_launch_arguments('traffic_light_detection',
        'traffic_light_v5_candidate.launch', original, unknown_grace_sec=.2)
    assert traffic == dict(original, preserve_unreadable_color_evidence=True,
                           stamped_vote_window_size=0, planner_compliance_observations=True)
    inherited = dict(planner_compliance_observations=False, stamped_vote_window_size=7)
    assert temporal_video_launch_arguments('traffic_light_detection', 'traffic_light_node.launch',
        inherited, unknown_grace_sec=.3) == dict(preserve_unreadable_color_evidence=True,
            stamped_vote_window_size=0, planner_compliance_observations=True)
    assert inherited == dict(planner_compliance_observations=False, stamped_vote_window_size=7)
    assert temporal_video_launch_arguments('bringup', 'sensing.launch', original,
        unknown_grace_sec=.2) == original
    assert temporal_video_launch_arguments('traffic_light_detection', 'traffic_light_node.launch',
        {}, unknown_grace_sec=0) == {'preserve_unreadable_color_evidence': False}
    assert temporal_video_launch_arguments('ajounice_planning_integration', 'planning.launch',
        original) == original
    assert temporal_video_launch_arguments('ajounice_planning_integration', 'planning.launch',
        {}, unknown_grace_sec=.3) == {'signal_compliance_unknown_grace_sec': .3}
    assert original == {'repo_root': '/repo'}
    for bad in (True, -.01, .300001, float('nan'), float('inf'), '0.2'):
        with pytest.raises(ValueError):
            temporal_video_launch_arguments('x', 'x', {}, unknown_grace_sec=bad)
    with pytest.raises(ValueError):
        temporal_video_launch_arguments('x', 'x', {}, unknown_grace_sec=.2, stamped_vote=7)


def test_seven_four_vote_parameters_are_explicit_scoped_and_bounded():
    original = {'device': '0', 'planner_compliance_observations': True}
    options = dict(stamped_vote=7, stamped_vote_max_age_sec=.9,
                   stamped_vote_startup_passthrough=True)
    result = temporal_video_launch_arguments('traffic_light_detection',
        'traffic_light_v5_candidate.launch', original, **options)
    assert result == dict(original, stamped_vote_window_size=7,
        stamped_vote_max_age_sec=.9, stamped_vote_startup_passthrough=True)
    assert original == {'device': '0', 'planner_compliance_observations': True}
    assert temporal_video_launch_arguments('bringup', 'sensing.launch', {}, **options) == {}
    for bad in (True, 0, -.1, float('nan'), .900001):
        with pytest.raises(ValueError):
            temporal_video_launch_arguments('x', 'x', {}, stamped_vote=7,
                                            stamped_vote_max_age_sec=bad)
    for window in (None, 0):
        with pytest.raises(ValueError):
            temporal_video_launch_arguments('x', 'x', {}, stamped_vote=window,
                                            stamped_vote_startup_passthrough=True)
    with pytest.raises(ValueError):
        temporal_video_launch_arguments('x', 'x', {}, stamped_vote=3,
                                        stamped_vote_max_age_sec=.9)
    with pytest.raises(ValueError):
        temporal_video_launch_arguments('x', 'x', {}, stamped_vote=7,
                                        stamped_vote_startup_passthrough='true')


def test_temporal_video_candidates_are_scoped_and_no_legacy_switch():
    original={'device':'0','planner_compliance_observations':True}
    out=temporal_video_launch_arguments('traffic_light_detection','traffic_light_v5_candidate.launch',
                                      original,2,3,True)
    assert out == dict(original,stamped_vote_window_size=3,publish_debug_image=True)
    assert original == {'device':'0','planner_compliance_observations':True}
    out=temporal_video_launch_arguments('ajounice_planning_integration','planning.launch',{},2,3,True)
    assert out == {'signal_compliance_required_green_observations':2}
    assert temporal_video_launch_arguments('bringup','x',{},2,3,True)=={}
    for bad in (True,0,4):
        with pytest.raises(ValueError):temporal_video_launch_arguments('x','x',{},bad,None)
from run_traffic_light_compliance_trial import (CANONICAL, SOURCE_TOPIC, active_fault_mode,
    fault_copy, fixture_relay_mode, validated_fixture_status, launch_arguments,
    require_trial_contract, run_owned_trial, signal_topic_conflicts, with_compliance_debug)
from run_traffic_light_compliance_trial import map_launch_arguments


def test_map_candidate_is_shared_without_overriding_policy_or_models():
    original = dict(device='0')
    assert map_launch_arguments('x', 'x', original, '/candidate', 'sha') == original
    assert map_launch_arguments('ajounice_planning_integration', 'planning.launch', original,
                                None, None) == original
    planning = map_launch_arguments('ajounice_planning_integration', 'planning.launch', original,
                                    '/candidate', 'sha')
    assert planning == dict(device='0', map_file='/candidate', map_sha256='sha')
    traffic = map_launch_arguments('traffic_light_detection', 'traffic_light_v5_candidate.launch',
                                   original, '/candidate', 'sha')
    assert traffic == dict(device='0', lanelet2_map_file='/candidate')
    prediction = map_launch_arguments('perception_object_pipeline', 'object_pipeline.launch',
                                      original, '/candidate', 'sha')
    assert prediction == dict(device='0', map_file='/candidate')
    assert original == dict(device='0')


def test_only_explicit_trial_launch_arguments_change():
    original = {'repo_root': '/repo', 'is_simulation': False}
    result = launch_arguments('ajounice_planning_integration', 'planning.launch', original, True, True, True)
    assert result['enable_signal_compliance_guard'] is True
    assert result['enable_yellow_passthrough'] is False
    assert result['signal_compliance_front_axle_offset_m'] == 3.0
    assert original == {'repo_root': '/repo', 'is_simulation': False}
    assert launch_arguments('bringup', 'localization.launch', original, True, True, True) == original
    assert launch_arguments('ajounice_planning_integration', 'planning.launch', original,
                            True, True, False, yellow_passthrough=True)['enable_yellow_passthrough'] is True


def test_production_mode_does_not_override_any_policy_launch_defaults():
    original = {'repo_root': '/repo', 'device': '0'}
    for package, filename in (('ajounice_planning_integration', 'planning.launch'),
                              ('traffic_light_detection', 'traffic_light_v5_candidate.launch')):
        selected = launch_arguments(package, filename, original, None, None, False)
        assert selected == original and selected is not original
        with pytest.raises(ValueError, match='production defaults'):
            launch_arguments(package, filename, original, None, None, True)


def test_traffic_trial_keeps_model_settings_and_uses_proxy_only_explicitly():
    original = {'device': '0', 'models_root': '/models'}
    result = launch_arguments('traffic_light_detection', 'traffic_light_v5_candidate.launch', original, False, True, True)
    assert result == dict(original, route_progress_guard=True, output_topic=SOURCE_TOPIC,
                          planner_compliance_observations=False,
                          preserve_unreadable_color_evidence=False)
    natural = launch_arguments('traffic_light_detection', 'traffic_light_v5_candidate.launch', original, False, False, False)
    assert natural == dict(original, route_progress_guard=False, planner_compliance_observations=False,
                           preserve_unreadable_color_evidence=False)
    assert 'output_topic' not in original


def test_trial_baselines_do_not_inherit_new_production_grace():
    for guarded in (False, True):
        planning = launch_arguments('ajounice_planning_integration', 'planning.launch',
                                    {}, guarded, True, False)
        assert planning['signal_compliance_unknown_grace_sec'] == 0.
        if not guarded:
            for key in ('defer_unseen_until_observed', 'defer_far_unseen_unknown',
                        'signal_compliance_enable_timed_yellow'):
                assert planning[key] is False
        traffic = launch_arguments('traffic_light_detection', 'traffic_light_node.launch',
                                   {}, guarded, True, False, temporal_direct=True)
        voted = temporal_video_launch_arguments('traffic_light_detection',
            'traffic_light_node.launch', traffic, stamped_vote=7)
        assert voted['preserve_unreadable_color_evidence'] is False


def test_fault_proxy_never_manufactures_color_or_freshness():
    lamp = SimpleNamespace(color=1, shape=2, confidence=.9)
    message = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=123),
        traffic_light_groups=[SimpleNamespace(traffic_light_group_id=201326, elements=[lamp]),
                              SimpleNamespace(traffic_light_group_id=201351, elements=[lamp])])
    assert fault_copy(message, 'none', 201326) is message
    assert fault_copy(message, 'drop', 201326) is None
    result = fault_copy(message, 'wrong_id', 201326)
    assert [group.traffic_light_group_id for group in result.traffic_light_groups] == [0, 201351]
    assert message.traffic_light_groups[0].traffic_light_group_id == 201326
    assert result.stamp == message.stamp
    assert result.traffic_light_groups[0].elements[0] == lamp


def test_visual_identity_candidate_is_explicit_bounded_and_perception_only():
    original={'device':'0'}
    value=launch_arguments('traffic_light_detection','traffic_light_v5_candidate.launch',
        original,True,True,False,temporal_direct=True,visual_continuation_sec=.3)
    assert value['visual_continuation_sec']==.3
    assert value['planner_compliance_observations'] is True
    assert original=={'device':'0'}
    planning=launch_arguments('ajounice_planning_integration','planning.launch',{},
        True,True,False,visual_continuation_sec=.3)
    assert 'visual_continuation_sec' not in planning
    for bad in (-.1,.31,float('nan'),float('inf'),True,'0.3'):
        with pytest.raises(ValueError):
            launch_arguments('traffic_light_detection','traffic_light_node.launch',{},
                True,True,False,visual_continuation_sec=bad)
    with pytest.raises(ValueError,match='production defaults'):
        launch_arguments('traffic_light_detection','traffic_light_node.launch',{},
            None,None,False,visual_continuation_sec=.3)


def test_injection_lease_never_silently_reverts_to_normal():
    state = dict(mode='none', received=False, terminal=False, receipt_monotonic=0.)
    assert active_fault_mode(state, 10.) == 'none'
    state.update(mode='drop', received=True, receipt_monotonic=10.)
    assert active_fault_mode(state, 10.5) == 'drop'
    with pytest.raises(RuntimeError): active_fault_mode(state, 10.501)
    state.update(mode='none', terminal=True)
    assert active_fault_mode(state, 20.) == 'none'


def test_covariance_stress_input_never_changes_planning_or_localization_inputs():
    topic='/evaluation/traffic_light_policy/covariance_odometry'
    traffic=launch_arguments('traffic_light_detection','traffic_light_node.launch',{},
        True,True,False,perception_odometry_topic=topic)
    assert traffic['odometry_topic']==topic
    for package,filename in (('bringup','localization.launch'),
                             ('ajounice_planning_integration','planning.launch')):
        result=launch_arguments(package,filename,{},True,True,False,
                                perception_odometry_topic=topic)
        assert 'odometry_topic' not in result
    for bad in ('/localization/kinematic_state',True,'/evaluation/traffic_light_policy/a b'):
        with pytest.raises(ValueError):
            launch_arguments('traffic_light_detection','traffic_light_node.launch',{},
                True,True,False,perception_odometry_topic=bad)
    with pytest.raises(ValueError):
        launch_arguments('traffic_light_detection','traffic_light_node.launch',{},
            None,None,False,perception_odometry_topic=topic)


def test_shared_topic_or_non_safety_case_is_rejected_before_native_helpers():
    assert signal_topic_conflicts({CANONICAL: ['/somebody_else']}) == {CANONICAL: ['/somebody_else']}
    cfg = dict(stack={'pose_source':'localization'}, cases=[object()],
               localization_safety={'enabled':True,'park_after_run':True})
    require_trial_contract(cfg)
    for key in ('enabled','park_after_run'):
        bad = dict(cfg, localization_safety=dict(cfg['localization_safety'], **{key:False}))
        with pytest.raises(ValueError): require_trial_contract(bad)


def test_owned_cleanup_covers_partial_helper_startup_before_graph_lock_release():
    events = []
    def prepare():
        events.append('observer_started')
        raise OSError('phase_controller_start_failed')
    with pytest.raises(OSError, match='phase_controller_start_failed'):
        run_owned_trial(prepare, lambda: events.append('campaign'),
                        lambda: events.append('cleanup_inside_lock'))
    events.append('owner_releases_lock')
    assert events == ['observer_started', 'cleanup_inside_lock', 'owner_releases_lock']


def test_owned_cleanup_preserves_campaign_result_and_also_handles_campaign_error():
    events = []
    assert run_owned_trial(lambda: events.append('prepare'), lambda: 23,
                           lambda: events.append('cleanup')) == 23
    assert events == ['prepare', 'cleanup']
    def fail():
        raise RuntimeError('campaign failed')
    with pytest.raises(RuntimeError, match='campaign failed'):
        run_owned_trial(lambda: None, fail, lambda: events.append('error_cleanup'))
    assert events[-1] == 'error_cleanup'


def test_synthetic_loss_never_implicitly_falls_back_to_real_vision():
    state = dict(received=False, started_monotonic=10.)
    assert fixture_relay_mode(state, 24.) == 'fixture'
    with pytest.raises(RuntimeError): fixture_relay_mode(state, 25.001)
    state.update(received=True, receipt_monotonic=30., handoff_real=False, error=False)
    assert fixture_relay_mode(state, 30.5) == 'fixture'
    with pytest.raises(RuntimeError): fixture_relay_mode(state, 30.501)
    with pytest.raises(RuntimeError): fixture_relay_mode(state, 29.9)
    state.update(handoff_real=True)
    assert fixture_relay_mode(state, 30.2) == 'real'
    state.update(error=True)
    with pytest.raises(RuntimeError): fixture_relay_mode(state, 30.2)


def test_fixture_handoff_requires_correct_scope_identity_and_observation_completion():
    body = dict(planner_only=True, group_id=201290, mode='loss-postfront',
                stage='fixture_complete', motion_stage_captured=True, handoff_real=True)
    assert validated_fixture_status(body, 201290, 'loss-postfront') == dict(handoff_real=True, error=False)
    for changed in (dict(planner_only=False), dict(group_id=201397), dict(mode='moving-yellow-close'),
                    dict(stage='probe'), dict(motion_stage_captured=False), dict(handoff_real='true')):
        with pytest.raises(ValueError): validated_fixture_status(dict(body, **changed), 201290, 'loss-postfront')
    error = dict(body, stage='not_applicable', handoff_real=False)
    assert validated_fixture_status(error, 201290, 'loss-postfront')['error']


def test_debug_logging_is_scoped_to_the_owned_planning_launch_only():
    command = ['roslaunch', 'package', 'file.launch']
    assert with_compliance_debug(command, 'traffic_light_detection', 'traffic_light_node.launch', '/tmp/debug') == command
    assert with_compliance_debug(command, 'ajounice_planning_integration', 'planning.launch', None) == command
    selected = with_compliance_debug(command, 'ajounice_planning_integration', 'planning.launch', '/tmp/a b')
    assert selected == command+['planning_rosconsole_config:=/tmp/a b']
    assert command == ['roslaunch', 'package', 'file.launch']
    with pytest.raises(TypeError):
        with_compliance_debug('roslaunch package file.launch', 'ajounice_planning_integration', 'planning.launch', '/tmp/debug')
