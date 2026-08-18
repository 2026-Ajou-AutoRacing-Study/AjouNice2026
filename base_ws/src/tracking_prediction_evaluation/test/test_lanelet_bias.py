import math

import pytest

from tracking_prediction_evaluation.lanelet_bias import (
    summarize_centerline_residuals,
    transform_local_point,
)


def test_summarize_centerline_residuals_preserves_vector_direction():
    rows = [
        {
            "correction_dx_m": -0.10,
            "correction_dy_m": 0.02,
            "distance_m": math.hypot(0.10, 0.02),
            "longitudinal_correction_m": -0.10,
            "lateral_correction_m": 0.02,
            "absolute_heading_error_deg": 1.0,
        },
        {
            "correction_dx_m": -0.08,
            "correction_dy_m": 0.00,
            "distance_m": 0.08,
            "longitudinal_correction_m": -0.08,
            "lateral_correction_m": 0.00,
            "absolute_heading_error_deg": 2.0,
        },
    ]

    summary = summarize_centerline_residuals(rows, input_count=4)

    assert summary["matched_count"] == 2
    assert summary["match_rate"] == pytest.approx(0.5)
    assert summary["global_correction_mean_vector_xy_m"] == pytest.approx(
        [-0.09, 0.01]
    )
    assert summary["global_correction_mean_magnitude_m"] == pytest.approx(
        math.hypot(0.09, 0.01)
    )


def test_transform_local_point_applies_rotation_before_translation():
    x, y = transform_local_point(
        2.0,
        0.0,
        translation_xyz=(10.0, 20.0, 0.0),
        yaw=math.pi / 2.0,
    )

    assert x == pytest.approx(10.0)
    assert y == pytest.approx(22.0)
