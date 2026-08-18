import math

from tracking_prediction_evaluation.pipeline_evaluator import (
    EgoPoint,
    ObjectEstimate,
    PredictedPathEstimate,
    _path_position,
    evaluate_pipeline,
    is_cv_stable_horizon,
    match_estimates,
    transform_sensor_estimates_to_map,
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


def test_multimodal_metrics_separate_top1_from_best_of_k():
    points = tuple(
        TrajectoryPoint(
            timestamp=float(timestamp),
            x=float(timestamp),
            y=0.0,
            z=0.0,
            yaw=0.0,
            longitudinal_velocity=1.0,
            lateral_velocity=0.0,
            longitudinal_acceleration=0.0,
            lateral_acceleration=0.0,
            class_name="vehicle",
            object_id=1,
            asset_name="test",
        )
        for timestamp in range(4)
    )
    segment = TrajectorySegment("vehicle-1", "vehicle", 1, "test", points)
    wrong_top1 = PredictedPathEstimate(
        points=((0.0, 0.0), (1.0, 10.0), (2.0, 10.0), (3.0, 10.0)),
        step_sec=1.0,
        confidence=0.6,
    )
    correct_second = PredictedPathEstimate(
        points=((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)),
        step_sec=1.0,
        confidence=0.4,
    )
    estimate = ObjectEstimate(
        object_id=b"track",
        x=0.0,
        y=0.0,
        predicted_path=wrong_top1.points,
        path_step_sec=wrong_top1.step_sec,
        predicted_paths=(wrong_top1, correct_second),
    )
    summary = evaluate_pipeline(
        (segment,),
        (),
        tracking_frames={},
        prediction_frames={0.0: (estimate,)},
        horizons=(3.0,),
    )
    metrics = summary["prediction"]["multimodal"]["horizons"]["3"]
    assert math.isclose(metrics["top1_fde_m"]["mean"], 10.0)
    assert math.isclose(metrics["min_fde_k_m"]["mean"], 0.0)
    assert math.isclose(metrics["min_ade_k_m"]["mean"], 0.0)
    assert math.isclose(metrics["best_fde_path_confidence"]["mean"], 0.4)
    assert math.isclose(metrics["brier_min_fde_m"]["mean"], 0.36)
    assert summary["prediction"]["multimodal"]["multi_path_object_rate"] == 1.0


def test_sensor_estimate_transform_preserves_pi_yaw_sign():
    ego = EgoPoint(1.0, 10.0, 20.0, math.pi / 2.0)
    estimate = ObjectEstimate(b"", 2.0, 0.0, yaw=math.pi)
    transformed = transform_sensor_estimates_to_map(
        (estimate,), ego, sensor_to_ego_x_m=1.0, sensor_to_ego_y_m=0.0
    )[0]
    assert math.isclose(transformed.x, 10.0, abs_tol=1.0e-9)
    assert math.isclose(transformed.y, 23.0, abs_tol=1.0e-9)
    assert math.isclose(transformed.yaw, -math.pi / 2.0, abs_tol=1.0e-9)


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
