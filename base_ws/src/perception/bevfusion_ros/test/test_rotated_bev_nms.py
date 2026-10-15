import math

from bevfusion_ros.rotated_bev_nms import (
    class_aware_rotated_bev_nms,
    rotated_bev_iou,
)


def detection(x, y, length, width, yaw, score, class_name="obstacle"):
    return {
        "center": [x, y, 0.0],
        "size": [length, width, 1.0],
        "yaw": yaw,
        "score": score,
        "class_name": class_name,
    }


def test_rotated_iou_is_symmetric_and_handles_rotation():
    first = detection(0.0, 0.0, 2.0, 1.0, 0.0, 0.9)
    same = detection(0.0, 0.0, 2.0, 1.0, 0.0, 0.8)
    perpendicular = detection(0.0, 0.0, 2.0, 1.0, math.pi / 2.0, 0.7)
    assert abs(rotated_bev_iou(first, same) - 1.0) < 1.0e-9
    assert abs(rotated_bev_iou(first, perpendicular) - 1.0 / 3.0) < 1.0e-9
    assert abs(
        rotated_bev_iou(first, perpendicular)
        - rotated_bev_iou(perpendicular, first)
    ) < 1.0e-9


def test_same_class_duplicate_keeps_higher_score():
    lower = detection(0.05, 0.0, 2.0, 1.0, 0.01, 0.2)
    higher = detection(0.0, 0.0, 2.0, 1.0, 0.0, 0.8)
    kept = class_aware_rotated_bev_nms([lower, higher], 0.5)
    assert kept == [higher]


def test_nearby_distinct_boxes_and_different_classes_survive():
    obstacle = detection(0.0, 0.0, 2.0, 1.0, 0.0, 0.9)
    nearby = detection(1.4, 0.0, 2.0, 1.0, 0.0, 0.8)
    vehicle = detection(0.0, 0.0, 2.0, 1.0, 0.0, 0.7, "vehicle")
    kept = class_aware_rotated_bev_nms([obstacle, nearby, vehicle], 0.5)
    assert kept == [obstacle, nearby, vehicle]


def test_equal_scores_are_deterministic_and_input_order_is_preserved():
    first = detection(0.0, 0.0, 2.0, 1.0, 0.0, 0.5)
    duplicate = detection(0.02, 0.0, 2.0, 1.0, 0.0, 0.5)
    distinct = detection(5.0, 0.0, 2.0, 1.0, 0.0, 0.4)
    kept = class_aware_rotated_bev_nms([first, duplicate, distinct], 0.5)
    assert kept == [first, distinct]


def test_invalid_threshold_is_rejected():
    try:
        class_aware_rotated_bev_nms([], 1.01)
    except ValueError:
        return
    raise AssertionError("invalid NMS threshold was accepted")
