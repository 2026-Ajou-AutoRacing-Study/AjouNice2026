#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

import generate_constant_curvature_lc as generator


class ConstantCurvatureLaneChangeTest(unittest.TestCase):
    def test_highway_design_has_reasonable_lateral_acceleration(self):
        kappa = 0.0014
        acceleration_170_kph = (170.0 / 3.6) ** 2 * kappa
        self.assertLess(acceleration_170_kph, 3.2)

    def test_length_solver_preserves_lane_width(self):
        length = generator.solve_length_for_width(3.5, 0.001)
        self.assertAlmostEqual(
            generator.lane_change_width(0.001, length), 3.5, places=9
        )
        self.assertGreater(length, 50.0)

    def test_kappa_filename_token_is_validator_safe(self):
        self.assertEqual(generator.kappa_filename_token(0.001434553), "0p001435")


if __name__ == "__main__":
    unittest.main()
