import math

from tracking_prediction_evaluation.pipeline_evaluator import (
    ObjectEstimate,
    _path_position,
    is_cv_stable_horizon,
    match_estimates,
)
from tracking_prediction_evaluation.trajectory import TrajectoryPoint, TrajectorySegment


def point(x, y, object_id):
    return TrajectoryPoint(
        timestamp=1.0,
        x=x,
        y=y,
        z=0.0,
        yaw=0.0,
        longitudinal_velocity=0.0,
        lateral_velocity=0.0,
        longitudinal_acceleration=0.0,
        lateral_acceleration=0.0,
        class_name="vehicle",
        object_id=object_id,
        asset_name="test",
    )


def test_assignment_uses_global_minimum_and_distance_gate():
    gt = [
        (TrajectorySegment("a", "vehicle", 1, "test", ()), point(0.0, 0.0, 1)),
        (TrajectorySegment("b", "vehicle", 2, "test", ()), point(10.0, 0.0, 2)),
    ]
    estimates = [
        ObjectEstimate(b"a", 0.2, 0.0),
        ObjectEstimate(b"b", 10.3, 0.0),
        ObjectEstimate(b"fp", 20.0, 0.0),
    ]
    matches, matched_gt, matched_estimates = match_estimates(gt, estimates, 2.0)
    assert [round(item[2], 6) for item in matches] == [0.2, 0.3]
    assert matched_gt == {0, 1}
    assert matched_estimates == {0, 1}


def test_predicted_path_interpolates_by_message_time_step():
    estimate = ObjectEstimate(
        b"track",
        0.0,
        0.0,
        predicted_path=((0.0, 0.0), (1.0, 0.0), (2.0, 2.0)),
        path_step_sec=1.0,
    )
    assert _path_position(estimate, 0.5) == (0.5, 0.0)
    assert _path_position(estimate, 1.5) == (1.5, 1.0)
    assert _path_position(estimate, 3.0) is None


def test_cv_stable_horizon_checks_complete_future_interval():
    points = tuple(
        TrajectoryPoint(
            timestamp=timestamp,
            x=timestamp,
            y=0.0,
            z=0.0,
            yaw=0.0,
            longitudinal_velocity=1.0,
            lateral_velocity=0.0,
            longitudinal_acceleration=acceleration,
            lateral_acceleration=0.0,
            class_name="vehicle",
            object_id=1,
            asset_name="test",
        )
        for timestamp, acceleration in (
            (0.0, 0.0),
            (0.2, 0.2),
            (0.4, 0.5),
            (0.6, 0.8),
            (0.8, 0.1),
            (1.0, 0.0),
        )
    )
    segment = TrajectorySegment("a", "vehicle", 1, "test", points)
    assert is_cv_stable_horizon(segment, 0.0, 0.4)
    assert not is_cv_stable_horizon(segment, 0.0, 1.0)
