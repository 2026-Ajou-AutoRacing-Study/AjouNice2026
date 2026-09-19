from bevfusion_ros.score_thresholds import (
    build_score_thresholds,
    filter_detections,
    server_score_floor,
)


def test_production_thresholds_filter_each_class_independently():
    fallback, thresholds = build_score_thresholds(0.10, 0.40, 0.15, 0.10)
    detections = [
        {"class_name": "Vehicle", "score": 0.39},
        {"class_name": "Vehicle", "score": 0.40},
        {"class_name": "Pedestrian", "score": 0.14},
        {"class_name": "Pedestrian", "score": 0.15},
        {"class_name": "Obstacle", "score": 0.10},
        {"class_name": "Other", "score": 0.09},
    ]

    kept = filter_detections(detections, fallback, thresholds)

    assert [(item["class_name"], item["score"]) for item in kept] == [
        ("Vehicle", 0.40),
        ("Pedestrian", 0.15),
        ("Obstacle", 0.10),
    ]
    assert server_score_floor(fallback, thresholds) == 0.10


def test_invalid_threshold_is_rejected():
    try:
        build_score_thresholds(0.10, 1.01, 0.15, 0.10)
    except ValueError as error:
        assert "vehicle" in str(error)
    else:
        raise AssertionError("invalid threshold was accepted")
