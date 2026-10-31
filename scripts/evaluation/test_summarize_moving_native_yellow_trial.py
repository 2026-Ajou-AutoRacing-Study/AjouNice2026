from summarize_moving_native_yellow_trial import transition_bounds


def test_transition_duration_uses_per_head_requested_and_confirmed_times():
    events = [dict(event='native_signal_requested', stage='moving_yellow', native_id='C1', requested_state=4, request_monotonic=10.),
              dict(event='native_signal_verified_actual', stage='moving_yellow', native_id='C1', actual_state=4, verified=True, response_monotonic=10.02),
              dict(event='native_signal_requested', stage='red_after_moving_yellow', native_id='C1', requested_state=1, request_monotonic=12.03),
              dict(event='native_signal_verified_actual', stage='red_after_moving_yellow', native_id='C1', actual_state=1, verified=True, response_monotonic=12.05)]
    r = transition_bounds(events, 'C1', 4)
    assert r['all_transition_records_present']
    low, high = r['explicit_transition_duration_bounds_sec']
    assert abs(low-2.01) < 1e-8 and abs(high-2.05) < 1e-8
    assert not transition_bounds(events, 'C2', 4)['all_transition_records_present']
    events[-1]['verified'] = False
    assert not transition_bounds(events, 'C1', 4)['all_transition_records_present']
