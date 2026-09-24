#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest

SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "gnss_imu_kinematic_state_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("gnss_imu_publisher", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GnssImuContractTest(unittest.TestCase):
    def test_missing_height_blocks_initial_output(self):
        self.assertEqual(MODULE.z_sample_state(False, 0.0, 0.5), "missing")

    def test_fresh_height_is_used(self):
        self.assertEqual(MODULE.z_sample_state(True, 0.49, 0.5), "fresh")

    def test_stale_height_is_held_instead_of_blocking_planar_state(self):
        self.assertEqual(MODULE.z_sample_state(True, 10.93, 0.5), "held")

    def test_future_height_is_diagnostic_hold_not_silent_freshness(self):
        self.assertEqual(MODULE.z_sample_state(True, -1.0, 0.5), "held")

    def test_non_finite_height_age_is_invalid(self):
        self.assertEqual(
            MODULE.z_sample_state(True, float("nan"), 0.5), "invalid"
        )

if __name__ == "__main__":
    unittest.main()
