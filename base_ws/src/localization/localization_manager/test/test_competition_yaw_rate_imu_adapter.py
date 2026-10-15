#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "competition_yaw_rate_imu_adapter.py"
)
SPEC = importlib.util.spec_from_file_location("competition_yaw_adapter", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_competition_clockwise_degrees_convert_to_ros_yaw_rate():
    assert math.isclose(
        MODULE.competition_yaw_rate_radps(30.0),
        -math.pi / 6.0,
        abs_tol=1.0e-12,
    )
    assert math.isclose(
        MODULE.competition_yaw_rate_radps(-45.0),
        math.pi / 4.0,
        abs_tol=1.0e-12,
    )
