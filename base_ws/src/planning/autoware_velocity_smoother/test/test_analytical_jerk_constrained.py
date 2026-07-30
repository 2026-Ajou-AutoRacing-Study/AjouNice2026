#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_velocity_smoother.analytical_jerk_constrained import (  # noqa: E402
    SmootherParameters,
    apply_engage_velocity_floor,
    minimum_deceleration_distance,
    smooth_velocity_profile,
)


def straight_positions(count, interval=0.5):
    return [(index * interval, 0.0, 0.0) for index in range(count)]


class AnalyticalJerkConstrainedSmootherTest(unittest.TestCase):
    def setUp(self):
        self.parameters = SmootherParameters(
            max_velocity_mps=10.0,
            max_acceleration_mps2=1.0,
            max_deceleration_mps2=2.5,
            max_jerk_mps3=1.0,
            max_deceleration_jerk_mps3=1.5,
            min_velocity_for_time_calculation_mps=1.0,
            stop_velocity_threshold_mps=0.01,
        )

    def test_stop_distance_grows_with_initial_velocity(self):
        slow = minimum_deceleration_distance(2.0, 0.0, 2.5, 1.5)
        fast = minimum_deceleration_distance(4.1666665, 0.0, 2.5, 1.5)
        self.assertGreater(slow, 0.0)
        self.assertGreater(fast, slow)

    def test_hard_stop_is_spread_over_multiple_trajectory_points(self):
        stop_index = 30
        caps = [4.1666665] * stop_index + [0.0] * 20
        result = smooth_velocity_profile(
            straight_positions(len(caps)),
            caps,
            initial_velocity_mps=4.1666665,
            parameters=self.parameters,
        )
        self.assertEqual(result.velocities[stop_index], 0.0)
        self.assertTrue(all(value == 0.0 for value in result.velocities[stop_index:]))
        self.assertLess(result.velocities[stop_index - 1], caps[stop_index - 1])
        self.assertTrue(
            any(0.0 < value < 4.1666665 for value in result.velocities[:stop_index])
        )
        self.assertTrue(
            all(
                value <= cap + 1.0e-6
                for value, cap in zip(result.velocities, caps)
            )
        )
        self.assertLess(abs(result.accelerations[stop_index - 2]), 2.6)
        self.assertLess(abs(result.accelerations[stop_index - 1]), 2.6)

    def test_standing_vehicle_accelerates_gradually(self):
        caps = [4.0] * 40
        result = smooth_velocity_profile(
            straight_positions(len(caps)),
            caps,
            initial_velocity_mps=0.0,
            parameters=self.parameters,
        )
        self.assertEqual(result.velocities[0], 0.0)
        self.assertGreater(result.velocities[5], 0.0)
        self.assertLess(result.velocities[1], caps[1])
        self.assertTrue(
            all(
                value <= cap + 1.0e-6
                for value, cap in zip(result.velocities, caps)
            )
        )

    def test_already_smoothed_cap_is_not_increased(self):
        caps = [4.0, 3.8, 3.4, 2.8, 2.0, 1.0, 0.0, 0.0]
        result = smooth_velocity_profile(
            straight_positions(len(caps)),
            caps,
            initial_velocity_mps=4.0,
            parameters=self.parameters,
        )
        self.assertTrue(
            all(
                value <= cap + 1.0e-6
                for value, cap in zip(result.velocities, caps)
            )
        )
        self.assertEqual(result.velocities[-1], 0.0)

    def test_engage_floor_does_not_override_a_near_stop(self):
        positions = straight_positions(20)
        caps = [4.0] * 8 + [0.0] * 12
        velocities = [0.0] * len(caps)
        output, engaged = apply_engage_velocity_floor(
            positions,
            velocities,
            caps,
            0.0,
            0.25,
            5.0,
            5.0,
            0.01,
        )
        self.assertFalse(engaged)
        self.assertEqual(output, tuple(velocities))

    def test_engage_floor_starts_a_clear_trajectory(self):
        positions = straight_positions(20)
        caps = [4.0] * len(positions)
        velocities = [0.0] * len(positions)
        output, engaged = apply_engage_velocity_floor(
            positions,
            velocities,
            caps,
            0.0,
            0.25,
            5.0,
            0.5,
            0.01,
        )
        self.assertTrue(engaged)
        self.assertEqual(output[0], 0.25)
        self.assertEqual(output[10], 0.25)
        self.assertEqual(output[11], 0.0)


if __name__ == "__main__":
    unittest.main()
