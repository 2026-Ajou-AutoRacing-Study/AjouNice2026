#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_path_optimizer.mpt_optimizer import MPTParameters, ModelPredictiveTrajectory
from autoware_path_optimizer.trajectory_utils import TrajectorySample, curvature


def path(count=80, lateral_noise=0.0):
    return [
        TrajectorySample(
            x=float(index),
            y=lateral_noise * math.sin(index * 0.7),
            yaw=0.0,
            longitudinal_velocity_mps=5.0 if index < count - 10 else 0.0,
        )
        for index in range(count)
    ]


def bounds(count=80, half_width=2.0):
    left = [(float(index), half_width, 0.0) for index in range(count)]
    right = [(float(index), -half_width, 0.0) for index in range(count)]
    return left, right


class ModelPredictiveTrajectoryTest(unittest.TestCase):
    def test_straight_path_is_preserved(self):
        source = path()
        left, right = bounds()
        result = ModelPredictiveTrajectory().optimize(source, 4.0, 0.0, left, right)
        self.assertTrue(result.success, result.status)
        self.assertLess(max(abs(point.y) for point in result.points), 1.0e-5)

    def test_noisy_path_has_lower_curvature_energy(self):
        source = path(lateral_noise=0.08)
        left, right = bounds()
        result = ModelPredictiveTrajectory().optimize(source, 2.0, source[2].y, left, right)
        self.assertTrue(result.success, result.status)
        source_energy = sum(value * value for value in curvature(source))
        output_energy = sum(value * value for value in curvature(result.points))
        self.assertLess(output_energy, source_energy)

    def test_offsets_stay_inside_vehicle_adjusted_bounds(self):
        source = path()
        left, right = bounds(half_width=1.3)
        parameters = MPTParameters(vehicle_width=2.0, soft_clearance_from_road=0.1)
        result = ModelPredictiveTrajectory(parameters).optimize(source, 1.0, 0.0, left, right)
        self.assertTrue(result.success, result.status)
        for offset, lower, upper in zip(result.offsets, result.lower_bounds, result.upper_bounds):
            self.assertGreaterEqual(offset + 1.0e-8, lower)
            self.assertLessEqual(offset - 1.0e-8, upper)

    def test_first_points_and_goal_are_fixed(self):
        source = path(count=30, lateral_noise=0.05)
        left, right = bounds(count=30)
        result = ModelPredictiveTrajectory().optimize(source, 0.0, source[0].y, left, right)
        self.assertTrue(result.success, result.status)
        self.assertTrue(all(abs(value) < 1.0e-8 for value in result.offsets[:3]))
        self.assertTrue(all(abs(value) < 1.0e-8 for value in result.offsets[-2:]))

    def test_invalid_bounds_fail_cleanly(self):
        result = ModelPredictiveTrajectory().optimize(path(), 0.0, 0.0, [], [])
        self.assertFalse(result.success)
        self.assertEqual(result.status, "invalid input")

    def test_corridor_narrower_than_vehicle_fails_safely(self):
        source = path()
        left, right = bounds(half_width=0.8)
        result = ModelPredictiveTrajectory().optimize(source, 0.0, 0.0, left, right)
        self.assertFalse(result.success)
        self.assertIn("narrower than vehicle", result.status)


if __name__ == "__main__":
    unittest.main()
