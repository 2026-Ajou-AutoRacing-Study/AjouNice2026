import math
import pytest
from tracking_prediction_evaluation.velocity_reference import axle_to_bbox_velocity
from tracking_prediction_evaluation.velocity_reference import pedestrian_message_velocity, pedestrian_velocity_source
from types import SimpleNamespace


def test_straight_velocity_is_unchanged():
    assert axle_to_bbox_velocity(10., 1., 0., 0., 1.3, .2) == (10., 1.)


def test_forward_offset_rotates_to_left_with_positive_yaw_rate():
    assert axle_to_bbox_velocity(10., 0., 0., .2, 1.5, 0.) == pytest.approx((10., .3))


def test_lateral_offset_affects_longitudinal_speed_and_map_rotation():
    assert axle_to_bbox_velocity(10., 0., math.pi/2, .2, 1.5, .5) == pytest.approx((-.3, 9.9))


def ped(t, vx=0.):
    return SimpleNamespace(timestamp=t, longitudinal_velocity=vx, lateral_velocity=0., yaw=math.pi/2)


def test_message_coordinate_frames_and_unit_conversion():
    assert pedestrian_message_velocity(ped(0., 3.6), 'kmh', 'body') == pytest.approx((0., 1.))
    assert pedestrian_message_velocity(ped(0., 3.6), 'kmh', 'morai-map', math.pi) == pytest.approx((-1., 0.))


def test_single_zero_and_real_stop_do_not_trigger_fallback():
    points = [ped(t, 1.) for t in (0., .1, .2)] + [ped(.3)]
    segment = SimpleNamespace(points=points)
    assert pedestrian_velocity_source(segment, points, (1., 0.)) == 'message'
    zero_history = [ped(t) for t in (.4, .5, .6)]
    assert pedestrian_velocity_source(segment, zero_history, (0., 0.)) == 'message'
    assert pedestrian_velocity_source(segment, zero_history, (1., 0.)) == 'position_fallback_moving_zero'


def test_nonfinite_message_does_not_become_gt():
    points = [ped(t, 1.) for t in (0., .1, .2)] + [ped(.3, float('nan'))]
    assert pedestrian_velocity_source(SimpleNamespace(points=points), points, (1., 0.)) == 'position_fallback_nonfinite'
