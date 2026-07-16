#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_path_smoother.elastic_band import ElasticBand
from autoware_path_smoother.trajectory_utils import (
    TrajectorySample,
    apply_input_velocity,
    extend_trajectory,
    set_zero_velocity_after_stop,
    insert_stop_point,
)


def sample(x, y, velocity=5.0):
    return TrajectorySample(x=float(x), y=float(y), yaw=0.0, longitudinal_velocity_mps=velocity)


class ElasticBandTest(unittest.TestCase):
    def test_straight_path_is_preserved(self):
        points = [sample(index, 0.0) for index in range(30)]
        output = ElasticBand().smooth(points, 5.0, 0.0)
        self.assertGreater(len(output), 20)
        self.assertLess(max(abs(point.y) for point in output), 1.0e-6)

    def test_smoothing_reduces_second_difference(self):
        points = [sample(index, 0.12 * math.sin(index * 0.8)) for index in range(40)]
        output = ElasticBand().smooth(points, 2.0, points[2].y)

        def roughness(values):
            return sum(
                (second.y - 2.0 * first.y + previous.y) ** 2
                for previous, first, second in zip(values[:-2], values[1:-1], values[2:])
            )

        self.assertLess(roughness(output), roughness(points))
        self.assertLessEqual(
            max(
                min(math.hypot(point.x - source.x, point.y - source.y) for source in points)
                for point in output
            ),
            0.75,
        )

    def test_first_and_goal_points_are_fixed(self):
        points = [sample(index, 0.05 * (-1) ** index) for index in range(20)]
        output = ElasticBand().smooth(points, 0.0, points[0].y)
        self.assertAlmostEqual(output[0].x, points[0].x, places=6)
        self.assertAlmostEqual(output[0].y, points[0].y, places=6)
        self.assertAlmostEqual(output[-1].x, points[-1].x, places=6)
        self.assertAlmostEqual(output[-1].y, points[-1].y, places=6)

    def test_stop_point_and_zero_tail_are_preserved(self):
        points = [sample(index, 0.0, 5.0 if index < 12 else 0.0) for index in range(25)]
        optimized = ElasticBand().smooth(points, 2.0, 0.0)
        output = apply_input_velocity(optimized, points)
        output = extend_trajectory(points, output, 0.5)
        output = set_zero_velocity_after_stop(output)
        first_zero = next(index for index, point in enumerate(output) if point.longitudinal_velocity_mps == 0.0)
        self.assertAlmostEqual(output[first_zero].x, 12.0, delta=0.51)
        self.assertTrue(all(point.longitudinal_velocity_mps == 0.0 for point in output[first_zero:]))

    def test_stop_can_be_inserted_between_samples(self):
        points = [sample(0.0, 0.0), sample(2.0, 0.0)]
        output = insert_stop_point(points, sample(1.0, 0.0, 0.0))
        self.assertEqual(len(output), 3)
        self.assertAlmostEqual(output[1].x, 1.0)
        self.assertEqual(output[1].longitudinal_velocity_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
