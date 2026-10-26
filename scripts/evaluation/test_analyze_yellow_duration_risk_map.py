import math

from analyze_yellow_duration_risk_map import risk_gate, stop_distance


def test_stop_model_branches_join_and_match_known_examples():
    assert stop_distance(0) == 0
    assert math.isclose(stop_distance(1.8-1e-7), stop_distance(1.8+1e-7), abs_tol=1e-6)
    assert math.isclose(stop_distance(30/3.6), 12.363703703703704)
    assert math.isclose(stop_distance(60/3.6), 36.391481481481485)


def test_duration_stopping_gate_and_delay_have_separate_effects():
    assert risk_gate(20., 50/3.6, 1.)
    assert not risk_gate(20., 50/3.6, 2.)
    assert not risk_gate(40., 50/3.6, 1.)  # Can brake: outside pass-distance gate.
    assert risk_gate(35., 60/3.6, 2.)
    assert not risk_gate(35., 60/3.6, 3.)
    assert not risk_gate(32., 60/3.6, 2.)
    assert risk_gate(32., 60/3.6, 2., .2)
    assert not risk_gate(.2, .5, 1.)  # Stopped/slow candidates excluded.
