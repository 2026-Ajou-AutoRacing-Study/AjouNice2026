import audit_traffic_braking_evidence as audit


def frame(t, speed, stop):
    return dict(write_ros_ns=round(t*1e9), image={'source_age_sec': .03}, telemetry={
        'planner_decision': {'receipt_age_sec': .01, 'source_age_sec': .01,
            'value': {'group_id': 201351, 'require_stop': stop, 'brake_m': 4.,
                      'target_m': 5., 'committed': False}},
        'ego': {'receipt_age_sec': .01, 'value': {'speed_kph': speed*3.6}},
        'command': {'value': {'brake': 1. if stop else 0.}}})


def test_audit_uses_measured_speed_integral_not_requested_braking_distance():
    frames = [frame(10, 4., False), frame(10.1, 4., True), frame(10.6, 2., True)]
    frames += [frame(11.1+i*.1, 0., True) for i in range(7)]
    result = audit.audit(frames, 201351)
    episode = result['stop_episodes'][0]
    assert abs(episode['measured_distance_velocity_integral_m']-2.) < 1e-6
    assert episode['calculated_brake_m'] == 4.
    assert episode['uninterrupted_stop']
    assert audit.audit(frames, 201326)['valid_samples'] == 0


def test_stale_evidence_cannot_be_counted_as_current_stop():
    frames = [frame(10, 4., False), frame(10.1, 4., True)]
    frames[1]['telemetry']['planner_decision']['receipt_age_sec'] = 1.
    assert audit.audit(frames, 201351)['valid_samples'] == 1
