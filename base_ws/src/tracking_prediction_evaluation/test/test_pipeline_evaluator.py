import math
from types import SimpleNamespace

from tracking_prediction_evaluation.pipeline_evaluator import (
    EgoPoint,
    ObjectEstimate,
    PredictedPathEstimate,
    _path_position,
    duration_seconds,
    evaluate_detection_tracking_yaw_flow,
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


def test_duration_seconds_accepts_ros1_and_ros2_layouts():
    ros1 = SimpleNamespace(secs=1, nsecs=250_000_000)
    ros2 = SimpleNamespace(sec=2, nanosec=500_000_000)
    assert math.isclose(duration_seconds(ros1), 1.25)
    assert math.isclose(duration_seconds(ros2), 2.5)


def test_detection_tracking_flow_reports_signed_stage_errors():
    gt_point = TrajectoryPoint(
        timestamp=0.0,
        x=0.0,
        y=0.0,
        z=0.0,
        yaw=0.0,
        longitudinal_velocity=0.0,
        lateral_velocity=0.0,
        longitudinal_acceleration=0.0,
        lateral_acceleration=0.0,
        class_name="vehicle",
        object_id=1,
        asset_name="test",
    )
    segment = TrajectorySegment("vehicle", "vehicle", 1, "test", (gt_point,))
    result = evaluate_detection_tracking_yaw_flow(
        (segment,),
        (EgoPoint(0.0, 0.0, 0.0, 0.0),),
        {0.0: (ObjectEstimate(b"", 1.0, 0.2, yaw=0.1),)},
        {0.0: (ObjectEstimate(b"track", 0.5, -0.1, yaw=0.05),)},
        sensor_to_ego_x_m=0.0,
        sensor_to_ego_y_m=0.0,
    )

    stages = result["state_error_by_stage"]
    assert math.isclose(stages["detection"]["signed_longitudinal_error_m"]["p50"], 1.0)
    assert math.isclose(stages["detection"]["signed_lateral_error_m"]["p50"], 0.2)
    assert math.isclose(stages["tracking"]["signed_longitudinal_error_m"]["p50"], 0.5)
    assert math.isclose(stages["tracking"]["signed_lateral_error_m"]["p50"], -0.1)
    assert result["paired_tracking_center_improvement_rate"] == 1.0
    assert result["paired_tracking_minus_detection_center_error_m"]["p50"] < 0.0


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
    top1_components = metrics["top1_fde_components_gt_heading"]
    assert math.isclose(top1_components["signed_longitudinal_m"]["p50"], 0.0)
    assert math.isclose(top1_components["signed_lateral_m"]["p50"], 10.0)
    min_k_components = metrics["min_fde_k_components_gt_heading"]
    assert math.isclose(min_k_components["absolute_longitudinal_m"]["p50"], 0.0)
    assert math.isclose(min_k_components["absolute_lateral_m"]["p50"], 0.0)
    assert summary["prediction"]["multimodal"]["multi_path_object_rate"] == 1.0


def test_origin_aligned_prediction_removes_only_current_translation_offset():
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
    shifted_path = PredictedPathEstimate(
        points=((1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)),
        step_sec=1.0,
        confidence=1.0,
    )
    estimate = ObjectEstimate(
        object_id=b"track",
        x=1.0,
        y=0.0,
        predicted_paths=(shifted_path,),
    )
    summary = evaluate_pipeline(
        (segment,),
        (),
        tracking_frames={},
        prediction_frames={0.0: (estimate,)},
        horizons=(3.0,),
        maximum_association_distance_m=2.0,
    )
    metrics = summary["prediction"]["multimodal"]["horizons"]["3"]
    assert math.isclose(metrics["top1_fde_m"]["mean"], 1.0)
    assert math.isclose(metrics["origin_aligned"]["top1_fde_m"]["mean"], 0.0)


def test_bbox_center_reference_exposes_absolute_tracking_metric():
    points = tuple(
        TrajectoryPoint(
            timestamp=float(timestamp),
            x=1.34 + float(timestamp),
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
        for timestamp in range(2)
    )
    segment = TrajectorySegment("vehicle", "vehicle", 1, "test", points)
    estimate = ObjectEstimate(b"track", 1.5, 0.0)
    summary = evaluate_pipeline(
        (segment,),
        (),
        tracking_frames={0.0: (estimate,)},
        prediction_frames={},
        gt_reference="bbox-center-calibrated-v1",
    )

    assert math.isclose(
        summary["tracking"]["bbox_center_absolute_distance_m"]["p50"],
        0.16,
        abs_tol=1.0e-9,
    )
    assert summary["tracking"]["object_status_reference_distance_m"]["count"] == 0


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
