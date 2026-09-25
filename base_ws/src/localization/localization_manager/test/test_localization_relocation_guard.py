#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "localization_relocation_guard.py"
SPEC = importlib.util.spec_from_file_location("localization_relocation_guard", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RelocationDetectorTest(unittest.TestCase):
    def test_single_outlier_does_not_trigger(self):
        detector = MODULE.RelocationDetector(5.0, 3, 5.0)
        self.assertFalse(detector.observe(20.0, 1.0))
        self.assertFalse(detector.observe(0.2, 1.1))
        self.assertEqual(detector.count, 0)

    def test_persistent_relocation_triggers_once(self):
        detector = MODULE.RelocationDetector(5.0, 3, 5.0)
        self.assertFalse(detector.observe(20.0, 1.0))
        self.assertFalse(detector.observe(20.0, 1.1))
        self.assertTrue(detector.observe(20.0, 1.2))
        self.assertFalse(detector.observe(20.0, 1.3))

    def test_cooldown_blocks_repeated_trigger(self):
        detector = MODULE.RelocationDetector(5.0, 2, 5.0)
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertTrue(detector.observe(6.0, 1.1))
        for stamp in (1.2, 1.3, 5.9, 6.0):
            self.assertFalse(detector.observe(6.0, stamp))
        self.assertFalse(detector.observe(6.0, 6.2))
        self.assertTrue(detector.observe(6.0, 6.3))

    def test_invalid_sample_clears_candidate(self):
        detector = MODULE.RelocationDetector(5.0, 2, 0.0)
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertFalse(detector.observe(math.nan, 1.1))
        self.assertFalse(detector.observe(6.0, 1.2))
        self.assertTrue(detector.observe(6.0, 1.3))

    def test_gnss_gap_clears_pre_outage_candidate(self):
        detector = MODULE.RelocationDetector(
            5.0, 3, 5.0, max_sample_gap_sec=0.5
        )
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertEqual(detector.count, 1)
        # The first post-outage sample starts a new consecutive sequence.
        self.assertFalse(detector.observe(6.0, 9.0))
        self.assertEqual(detector.count, 1)
        self.assertFalse(detector.observe(6.0, 9.1))
        self.assertTrue(detector.observe(6.0, 9.2))

    def test_non_monotonic_stamp_clears_candidate(self):
        detector = MODULE.RelocationDetector(
            5.0, 3, 5.0, max_sample_gap_sec=0.5
        )
        self.assertFalse(detector.observe(6.0, 2.0))
        self.assertFalse(detector.observe(6.0, 1.9))
        self.assertEqual(detector.count, 1)

    def test_parameter_validation(self):
        with self.assertRaises(ValueError):
            MODULE.RelocationDetector(0.0, 3, 5.0)
        with self.assertRaises(ValueError):
            MODULE.RelocationDetector(5.0, 0, 5.0)
        with self.assertRaises(ValueError):
            MODULE.RelocationDetector(5.0, 3, 5.0, max_sample_gap_sec=0.0)


if __name__ == "__main__":
    unittest.main()
