"""Offline checks only: no ROS, UDP, native calls, or vehicle control."""
import math

import pytest

from control_traffic_light_policy_trial import (
    TrialError, coordinated_native_cycle, experiment_stages, moving_front_geometry, moving_trigger_gate)


G = dict(group_id=201326, native_ids=['C1256W000052'], stop_line_xy=[[20., -2.], [20., 2.]])


def sample(x=9., speed=4., authorized=True, source=10000000000, gear=4):
    r = {k: dict(receipt_monotonic=10., source_stamp_ns=source) for k in ('odom', 'gear', 'ctrl', 'signal')}
    r['odom'].update(position_xyz=[x, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.],
                     velocity_xyz=[speed, 0., 0.], frame='map')
    r['gear']['value'] = gear
    r['signal']['authorized'] = authorized
    return r


def test_moving_program_has_real_route_native_colors_and_bounded_single_duration():
    for gid, yellow, green in [(201351, 4, 16), (201365, 4, 16), (201290, 4, 48), (201326, 5, 33)]:
        stages = experiment_stages(gid, 'moving-native-yellow', (2.,))
        assert [(s['color'], s['seconds']) for s in stages] == [(yellow, 2.), (1, 3.), (green, None)]
    for durations in [(1., 2.), (5.,), (), (1.5,)]:
        with pytest.raises(ValueError): experiment_stages(201326, 'moving-native-yellow', durations)


def test_gate_needs_motion_D_and_fresh_real_authorization():
    r = sample()
    gate = moving_trigger_gate(r, G, 10., 10000000000, 8.)
    assert gate['ready'] and gate['geometry']['front_distance_m'] == 8.
    for change in [dict(authorized=False), dict(speed=.1), dict(gear=1), dict(x=8.)]:
        assert not moving_trigger_gate(sample(**change), G, 10., 10000000000, 8.)['ready']
    r['signal']['receipt_monotonic'] = 9.
    assert not moving_trigger_gate(r, G, 10., 10000000000, 8.)['ready']
    with pytest.raises(TrialError): moving_trigger_gate(sample(source=1), G, 10., 10000000000, 8.)


def test_geometry_is_front_axle_and_finite_segment_not_unbounded_line():
    assert moving_front_geometry(sample()['odom'], G)['front_distance_m'] == 8.
    r = sample();r['odom']['position_xyz'][1] = 10.
    assert not moving_front_geometry(r['odom'], G)['ready']
    r = sample();r['odom']['quaternion_xyzw'] = [0., 0., math.sin(math.pi/4), math.cos(math.pi/4)]
    assert not moving_front_geometry(r['odom'], G)['ready']


def test_multi_head_native_set_waits_for_each_frame_and_reads_both_without_synthetic_ack():
    events = []
    def send(identifier, color):
        events.append(('SET', identifier, color))
    def read(identifier):
        events.append(('GET', identifier))
        return 48 if identifier == 'C016' else 1
    result = coordinated_native_cycle(['C016', 'C018'], 48, send, read,
                                      frame_wait=lambda t: events.append(('WAIT', t)))
    assert events == [('SET', 'C016', 48), ('WAIT', .1), ('SET', 'C018', 48), ('WAIT', .1),
                      ('GET', 'C016'), ('GET', 'C018')]
    assert result == {'C016': 48, 'C018': 1}  # Never manufacture agreement.
    events.clear()
    coordinated_native_cycle(['A077'], 16, send, read, frame_wait=lambda t: events.append(('WAIT', t)))
    assert events == [('SET', 'A077', 16), ('GET', 'A077')]
