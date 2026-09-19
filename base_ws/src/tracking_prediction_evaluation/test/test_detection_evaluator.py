import math

from tracking_prediction_evaluation.detection_evaluator import (
    DetectionEstimate,
    EvaluationFrame,
    evaluate_detection_frames,
)
from tracking_prediction_evaluation.pipeline_evaluator import EgoPoint
from tracking_prediction_evaluation.trajectory import TrajectoryPoint, TrajectorySegment


def _gt(x, object_id):
    point = TrajectoryPoint(
        timestamp=0.0,
        x=x,
        y=0.0,
        z=0.0,
        yaw=0.0,
        longitudinal_velocity=0.0,
        lateral_velocity=0.0,
        longitudinal_acceleration=0.0,
        lateral_acceleration=0.0,
        class_name="vehicle",
        object_id=object_id,
        asset_name="car",
        length=4.8,
        width=2.0,
        height=1.5,
    )
    return TrajectorySegment(str(object_id), "vehicle", object_id, "car", (point,)), point


def _prediction(x, score):
    return DetectionEstimate("vehicle", score, x, 0.0, 0.0, 4.8, 2.0, 1.5, 0.0)


def test_online_detection_metrics_rank_tp_before_fp():
    frame = EvaluationFrame(
        0.0,
        EgoPoint(0.0, 0.0, 0.0),
        (_gt(5.0, 1),),
        (_prediction(5.1, 0.9), _prediction(15.0, 0.2)),
    )
    report = evaluate_detection_frames((frame,), class_names=("vehicle",))
    vehicle = report["overall"]["classes"]["vehicle"]
    assert vehicle["gt"] == 1
    assert vehicle["predictions"] == 2
    assert vehicle["mean_ap"] > 0.95
    assert vehicle["bev_iou_ap"]["0.5"]["ap"] > 0.95
    assert vehicle["production_operating_point_1m"]["tp"] == 1
    assert vehicle["production_operating_point_1m"]["fp"] == 1
    assert math.isclose(vehicle["tp_quality_2m"]["mATE_m"], 0.1)


def test_distance_bins_do_not_count_objects_outside_bin():
    frame = EvaluationFrame(
        0.0,
        EgoPoint(0.0, 0.0, 0.0),
        (_gt(5.0, 1), _gt(15.0, 2)),
        (_prediction(5.1, 0.9), _prediction(15.1, 0.8)),
    )
    report = evaluate_detection_frames((frame,), class_names=("vehicle",))
    assert report["distance_bins_m"]["0-10"]["classes"]["vehicle"]["gt"] == 1
    assert report["distance_bins_m"]["10-20"]["classes"]["vehicle"]["gt"] == 1
    assert report["distance_bins_m"]["20-30"]["classes"]["vehicle"]["gt"] == 0
