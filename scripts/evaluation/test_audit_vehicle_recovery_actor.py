from audit_vehicle_recovery_actor import actor_at_birth, fixed_actor_rows, paired_fixed_actor, select_lineage


def track(identity, initialized, center, associated=True, measured=100.):
    return dict(id=identity, initialization_time=initialized, confirmed=True,
                center=center, velocity=[4., 0.], associated=associated, last_measurement_time=measured)


def test_lineage_resolves_same_initialization_when_global_id_counter_changed():
    rows = [dict(stamp=100., tracks=[track(8, 95., [1., 0.]), track(9, 95., [30., 0.])])]
    assert select_lineage(rows, 95., [1., 0.], 100.) == (8, 95.)
    assert select_lineage(rows, 96., [1., 0.], 100.) is None


def test_fixed_actor_keeps_old_outside_two_meter_gate_and_counts_both_outputs():
    series = ([99.9, 100., 100.1], [dict(center=[.0, 0.]), dict(center=[.4, 0.]), dict(center=[.8, 0.])])
    rows = [dict(stamp=100., tracks=[track(0, 95., [6., 0.], False, 99.8), track(1, 99.9, [.4, 0.])])]
    values = fixed_actor_rows(rows, series, (0, 95.), (1, 99.9), 99.9, 100.1)
    assert values[0]['concurrent_old_fresh']
    assert values[0]['outputs'][0]['current_error_m'] > 2.
    assert values[0]['primary']['id'] == 1


def test_actor_birth_requires_clear_nearby_gt_and_pairing_not_current_distance():
    gt = dict(gt=[([100.], [dict(class_name='vehicle', center=[0., 0.])]),
                  ([100.], [dict(class_name='vehicle', center=[.2, 0.])])])
    event = dict(stamp=100., raw_measurement=dict(center=[.05, 0.]))
    assert actor_at_birth(gt, event) is None
    gt['gt'].pop()
    assert actor_at_birth(gt, event) == 0
    baseline = [dict(stamp=100., primary=dict(id=1, current_error_m=.1, velocity_error_mps=1.))]
    candidate = [dict(stamp=100., primary=dict(id=2, current_error_m=6., velocity_error_mps=7.))]
    result = paired_fixed_actor(baseline, candidate)
    assert result['paired'] == 1
    assert result['current_error_m']['delta']['maximum'] == 5.9
