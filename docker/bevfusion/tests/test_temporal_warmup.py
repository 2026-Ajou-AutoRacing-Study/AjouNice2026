import numpy as np
import pytest

from app.bevfusion_model import build_warmup_points


def test_single_frame_warmup_contract():
    points = build_warmup_points(4, 1, point_count=11)
    assert points.shape == (11, 4)
    assert points.dtype == np.float32


def test_three_sweep_warmup_contract():
    points = build_warmup_points(5, 3, point_count=12)
    assert points.shape == (12, 5)
    assert np.unique(points[:, 4]).tolist() == pytest.approx([0.0, 0.1, 0.2])
    assert np.count_nonzero(points[:, 4] == 0.0) == 4


def test_temporal_warmup_rejects_missing_age_channel():
    with pytest.raises(ValueError, match="age feature"):
        build_warmup_points(4, 3)
