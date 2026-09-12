import numpy as np

from morai_pointpainting_ros.score_thresholds import build_score_thresholds


def test_class_thresholds_follow_one_based_openpcdet_labels():
    labels = np.asarray([1, 2, 3, 4])

    thresholds = build_score_thresholds(
        labels,
        ("Vehicle", "Pedestrian", "Obstacle"),
        0.1,
        {"vehicle": 0.4, "pedestrian": 0.15, "obstacle": 0.1},
    )

    np.testing.assert_allclose(thresholds, [0.4, 0.15, 0.1, 0.1])


def test_unknown_class_names_keep_the_legacy_fallback():
    thresholds = build_score_thresholds(
        np.asarray([1, 2]),
        ("Vehicle", "Cyclist"),
        0.2,
        {"vehicle": 0.4},
    )

    np.testing.assert_allclose(thresholds, [0.4, 0.2])
