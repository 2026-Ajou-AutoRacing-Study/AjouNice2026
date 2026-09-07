import math

from tracking_prediction_evaluation.pipeline_evaluator import EgoPoint, ObjectEstimate
from tracking_prediction_evaluation.timestamp_alignment import (
    AlignmentSample,
    build_alignment_samples,
    evaluate_alignment_shift,
    sweep_alignment_shifts,
)
from tracking_prediction_evaluation.trajectory import (
    TrajectoryPoint,
    TrajectorySegment,
)


def moving_segment(key, speed):
    points = tuple(
        TrajectoryPoint(
            timestamp=index * 0.05,
            x=speed * index * 0.05,
            y=0.0,
            z=0.0,
            yaw=0.0,
            longitudinal_velocity=speed,
            lateral_velocity=0.0,
            longitudinal_acceleration=0.0,
            lateral_acceleration=0.0,
            class_name="vehicle",
            object_id=int(speed),
            asset_name="test",
        )
        for index in range(41)
    )
    return TrajectorySegment(key, "vehicle", int(speed), "test", points)


def test_shift_sweep_recovers_known_state_reference_offset():
    timestamp = 1.0
    samples = []
    for speed in (5.0, 10.0):
        segment = moving_segment(str(speed), speed)
        estimate = ObjectEstimate(
            object_id=str(speed).encode(),
            x=speed * (timestamp + 0.1),
            y=0.0,
            yaw=0.0,
        )
        samples.append(AlignmentSample(timestamp, segment, estimate))

    zero = evaluate_alignment_shift(samples, 0.0)
    assert math.isclose(
        zero["longitudinal_error_vs_speed"]["time_slope_sec"],
        0.1,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        zero["longitudinal_error_vs_speed"]["spatial_intercept_m"],
        0.0,
        abs_tol=1.0e-9,
    )

    result = sweep_alignment_shifts(samples, (-0.1, 0.0, 0.1, 0.2))
    best = result["best"]["euclidean_mean"]
    assert math.isclose(best["shift_sec"], 0.1, abs_tol=1.0e-9)
    assert math.isclose(best["euclidean_error_m"]["mean"], 0.0, abs_tol=1.0e-9)


def test_sensor_frame_shift_moves_gt_and_ego_together():
    timestamp = 1.0
    offset = 0.1
    sensor_x = 0.855
    segment = moving_segment("vehicle", 10.0)
    ego_points = tuple(
        EgoPoint(index * 0.05, 2.0 * index * 0.05, 0.0, 0.0)
        for index in range(41)
    )
    relative_x = (10.0 - 2.0) * (timestamp + offset) - sensor_x
    sample = AlignmentSample(
        timestamp,
        segment,
        ObjectEstimate(b"vehicle", relative_x, 0.0, 0.0),
        estimate_is_sensor_frame=True,
        sensor_to_ego_x_m=sensor_x,
    )

    result = sweep_alignment_shifts(
        (sample,),
        (-0.1, 0.0, 0.1, 0.2),
        ego_points=ego_points,
    )
    best = result["best"]["euclidean_mean"]
    assert math.isclose(best["shift_sec"], offset, abs_tol=1.0e-9)
    assert math.isclose(best["euclidean_error_m"]["mean"], 0.0, abs_tol=1.0e-9)


def test_sensor_frame_range_filter_preserves_source_estimate_index():
    points = tuple(
        TrajectoryPoint(
            timestamp=index * 0.1,
            x=10.0,
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
        for index in range(21)
    )
    segment = TrajectorySegment("vehicle", "vehicle", 1, "test", points)
    ego_points = tuple(EgoPoint(index * 0.1, 0.0, 0.0, 0.0) for index in range(21))
    outside = ObjectEstimate(b"outside", 100.0, 0.0)
    matching = ObjectEstimate(b"matching", 10.0, 0.0)
    samples, _ = build_alignment_samples(
        (segment,),
        ego_points,
        {1.0: (outside, matching)},
        estimates_are_sensor_frame=True,
        sensor_to_ego_x_m=0.0,
        maximum_range_m=30.0,
    )
    assert len(samples) == 1
    assert samples[0].estimate.object_id == b"matching"
