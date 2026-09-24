#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "smooth_control_state_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("smooth_control_state", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SmoothControlStateTest(unittest.TestCase):
    def test_alignment_round_trip(self):
        local = (12.0, -4.0, 0.7)
        expected = (103.0, 51.0, -0.2)
        alignment = MODULE.planar_alignment(expected, local)
        actual = MODULE.compose_planar(alignment, local)
        for lhs, rhs in zip(actual, expected):
            self.assertAlmostEqual(lhs, rhs, places=9)

    def test_exponential_step_is_frame_rate_independent(self):
        first = MODULE.PlanarAlignmentSmoother(1.0, 1.0)
        second = MODULE.PlanarAlignmentSmoother(1.0, 1.0)
        first.reset((0.0, 0.0, 0.0), 0.0)
        second.reset((0.0, 0.0, 0.0), 0.0)
        for index in range(1, 11):
            first.update((1.0, 0.0, 0.0), index * 0.1)
        for index in range(1, 51):
            second.update((1.0, 0.0, 0.0), index * 0.02)
        self.assertAlmostEqual(first.value[0], second.value[0], places=12)

    def test_translation_rate_limit(self):
        smoother = MODULE.PlanarAlignmentSmoother(
            0.0, 0.0, max_translation_rate_mps=0.5
        )
        smoother.reset((0.0, 0.0, 0.0), 0.0)
        value = smoother.update((10.0, 0.0, 0.0), 0.1)
        self.assertAlmostEqual(value[0], 0.05)

    def test_yaw_uses_shortest_wrapped_delta(self):
        smoother = MODULE.PlanarAlignmentSmoother(0.0, 0.0)
        smoother.reset((0.0, 0.0, math.radians(179.0)), 0.0)
        value = smoother.update((0.0, 0.0, math.radians(-179.0)), 0.1)
        self.assertAlmostEqual(abs(math.degrees(value[2])), 179.0)

    def test_long_gap_reinitializes_without_slow_catch_up(self):
        smoother = MODULE.PlanarAlignmentSmoother(1.0, 1.0, reset_gap_sec=0.5)
        smoother.reset((0.0, 0.0, 0.0), 0.0)
        value = smoother.update((2.0, 3.0, 0.4), 1.0)
        self.assertEqual(value, (2.0, 3.0, 0.4))


if __name__ == "__main__":
    unittest.main()
