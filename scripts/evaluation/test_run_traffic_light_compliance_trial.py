from types import SimpleNamespace

import pytest
from run_traffic_light_compliance_trial import (CANONICAL, SOURCE_TOPIC, active_fault_mode,
    fault_copy, launch_arguments, require_trial_contract, run_owned_trial, signal_topic_conflicts)


def test_only_explicit_trial_launch_arguments_change():
    original = {'repo_root': '/repo', 'is_simulation': False}
    result = launch_arguments('ajounice_planning_integration', 'planning.launch', original, True, True, True)
    assert result['enable_signal_compliance_guard'] is True
    assert result['signal_compliance_front_axle_offset_m'] == 3.0
    assert original == {'repo_root': '/repo', 'is_simulation': False}
    assert launch_arguments('bringup', 'localization.launch', original, True, True, True) == original


def test_traffic_trial_keeps_model_settings_and_uses_proxy_only_explicitly():
    original = {'device': '0', 'models_root': '/models'}
    result = launch_arguments('traffic_light_detection', 'traffic_light_v5_candidate.launch', original, False, True, True)
    assert result == dict(original, route_progress_guard=True, output_topic=SOURCE_TOPIC,
                          planner_compliance_observations=False)
    natural = launch_arguments('traffic_light_detection', 'traffic_light_v5_candidate.launch', original, False, False, False)
    assert natural == dict(original, route_progress_guard=False, planner_compliance_observations=False)
    assert 'output_topic' not in original


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


def test_injection_lease_never_silently_reverts_to_normal():
    state = dict(mode='none', received=False, terminal=False, receipt_monotonic=0.)
    assert active_fault_mode(state, 10.) == 'none'
    state.update(mode='drop', received=True, receipt_monotonic=10.)
    assert active_fault_mode(state, 10.5) == 'drop'
    with pytest.raises(RuntimeError): active_fault_mode(state, 10.501)
    state.update(mode='none', terminal=True)
    assert active_fault_mode(state, 20.) == 'none'


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
