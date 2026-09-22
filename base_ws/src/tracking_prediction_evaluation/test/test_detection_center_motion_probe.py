#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "analyze_detection_center_motion_probe.py"
SPEC = importlib.util.spec_from_file_location("center_motion_probe", MODULE_PATH)
PROBE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PROBE
SPEC.loader.exec_module(PROBE)


class Point:
    def __init__(self, timestamp, x, y):
        self.timestamp = timestamp
        self.x = x
        self.y = y


class DetectionCenterMotionProbeTest(unittest.TestCase):
    def test_pair_error_is_zero_for_matching_motion(self):
        previous = PROBE.Observation(0.0, 2.0, 1.0, "car")
        current = PROBE.Observation(0.5, 7.0, 1.0, "car")
        row = PROBE.pair_error(
            previous,
            current,
            Point(0.0, 0.0, 0.0),
            Point(0.5, 5.0, 0.0),
        )
        self.assertAlmostEqual(row["vector_error"], 0.0)
        self.assertAlmostEqual(row["speed_error"], 0.0)

    def test_choose_past_prefers_target_baseline(self):
        observations = [
            PROBE.Observation(0.0, 0.0, 0.0),
            PROBE.Observation(0.1, 0.0, 0.0),
            PROBE.Observation(0.2, 0.0, 0.0),
        ]
        selected = PROBE.choose_past(observations, 0.55, 0.3, 0.45, 0.6)
        self.assertEqual(selected.timestamp, 0.1)

    def test_velocity_error_uses_same_pair_reference(self):
        row = PROBE.velocity_error(
            10.0,
            0.0,
            Point(0.0, 0.0, 0.0),
            Point(0.5, 5.0, 0.0),
            0.5,
        )
        self.assertAlmostEqual(row["vector_error"], 0.0)
        self.assertAlmostEqual(row["speed_error"], 0.0)

    def test_linear_fit_residual_rejects_center_outlier(self):
        straight = [
            PROBE.Observation(0.0, 0.0, 0.0),
            PROBE.Observation(0.5, 5.0, 0.0),
            PROBE.Observation(1.0, 10.0, 0.0),
        ]
        outlier = straight + [PROBE.Observation(1.5, 15.0, 1.0)]
        self.assertAlmostEqual(PROBE.linear_fit_residual_m(straight), 0.0)
        self.assertGreater(PROBE.linear_fit_residual_m(outlier), 0.2)

    def test_axial_heading_treats_pi_flip_as_equivalent(self):
        self.assertAlmostEqual(
            PROBE.axial_heading_error_deg(-10.0, 0.0, 0.0), 0.0
        )


if __name__ == "__main__":
    unittest.main()
