#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_velocity_smoother.analytical_jerk_constrained import (  # noqa: E402
    SmootherParameters,
    apply_engage_velocity_floor,
    apply_lateral_acceleration_limits,
    interpolate_values_at_stations,
    minimum_deceleration_distance,
    nearest_index_2d,
    project_motion_state,
    resample_profile_inputs,
    select_initial_state,
    smooth_velocity_profile,
)


def straight_positions(count, interval=0.5):
    return [(index * interval, 0.0, 0.0) for index in range(count)]


def arc_positions(radius, count):
    return [
        (
            radius * math.cos(index * math.pi / (2.0 * (count - 1))),
            radius * math.sin(index * math.pi / (2.0 * (count - 1))),
            0.0,
        )
        for index in range(count)
    ]


def straight_then_arc_positions(radius, arc_count):
    straight = [(float(x), 0.0, 0.0) for x in range(-40, 0, 5)]
    arc = [
        (
            radius * math.cos(-math.pi / 2.0 + index * math.pi / (2.0 * (arc_count - 1))),
            radius + radius * math.sin(
                -math.pi / 2.0 + index * math.pi / (2.0 * (arc_count - 1))
            ),
            0.0,
        )
        for index in range(arc_count)
    ]
    return straight + arc


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

    def test_nearest_index_2d_returns_closest_position(self):
        index, distance = nearest_index_2d(
            [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
            (1.6, 0.2, 0.0),
        )
        self.assertEqual(index, 1)
        self.assertLess(distance, 0.5)

    def test_projected_state_interpolates_between_previous_points(self):
        state = project_motion_state(
            [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
            [4.0, 2.0],
            [-0.2, -0.6],
            (0.5, 0.2, 0.0),
        )
        self.assertAlmostEqual(state.velocity_mps, 3.5)
        self.assertAlmostEqual(state.acceleration_mps2, -0.3)
        self.assertAlmostEqual(state.distance_m, 0.2)
        self.assertAlmostEqual(state.segment_ratio, 0.25)

    def test_resampling_keeps_original_stations_and_stop_cap(self):
        positions = straight_positions(3, interval=0.5)
        caps = [3.0, 3.0, 0.0]
        result = resample_profile_inputs(positions, caps, 0.1)
        self.assertEqual(len(result.positions), 11)
        self.assertEqual(result.input_stations, (0.0, 0.5, 1.0))
        recovered_caps = interpolate_values_at_stations(
            result.stations,
            result.velocity_caps,
            result.input_stations,
        )
        self.assertEqual(recovered_caps, tuple(caps))

    def test_initial_state_uses_previous_plan_when_tracking_is_consistent(self):
        state = select_initial_state(
            current_velocity_mps=4.2,
            previous_velocity_mps=4.0,
            previous_acceleration_mps2=-0.3,
            replan_velocity_deviation_mps=1.0,
        )
        self.assertEqual(state.source, "previous_plan")
        self.assertEqual(state.velocity_mps, 4.0)
        self.assertEqual(state.acceleration_mps2, -0.3)

    def test_initial_state_uses_current_velocity_on_large_deviation(self):
        state = select_initial_state(
            current_velocity_mps=6.0,
            previous_velocity_mps=4.0,
            previous_acceleration_mps2=-0.3,
            replan_velocity_deviation_mps=1.0,
        )
        self.assertEqual(state.source, "current_replan")
        self.assertEqual(state.velocity_mps, 6.0)
        self.assertEqual(state.acceleration_mps2, -0.3)

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

    def test_deceleration_uses_normal_limit_and_infeasible_modes(self):
        parameters = SmootherParameters(
            max_velocity_mps=10.0,
            max_acceleration_mps2=1.0,
            max_deceleration_mps2=1.0,
            max_jerk_mps3=1.0,
            max_deceleration_jerk_mps3=1.0,
            limit_max_deceleration_mps2=2.5,
            limit_max_deceleration_jerk_mps3=1.5,
        )

        def profile_for_stop_distance(distance_m):
            positions = straight_positions(int(distance_m * 10.0) + 1, interval=0.1)
            caps = [2.78] * (len(positions) - 1) + [0.0]
            return smooth_velocity_profile(
                positions,
                caps,
                initial_velocity_mps=2.78,
                parameters=parameters,
            )

        normal = profile_for_stop_distance(6.0)
        limited = profile_for_stop_distance(4.5)
        infeasible = profile_for_stop_distance(3.0)
        self.assertEqual(normal.deceleration_mode, "normal")
        self.assertTrue(normal.stop_feasible)
        self.assertEqual(limited.deceleration_mode, "limit")
        self.assertTrue(limited.stop_feasible)
        self.assertEqual(infeasible.deceleration_mode, "infeasible")
        self.assertFalse(infeasible.stop_feasible)
        self.assertLess(infeasible.velocities[0], 2.78)

    def test_receding_horizon_preserves_deceleration_state(self):
        parameters = SmootherParameters(
            max_velocity_mps=10.0,
            max_acceleration_mps2=1.0,
            max_deceleration_mps2=1.0,
            max_jerk_mps3=1.0,
            max_deceleration_jerk_mps3=1.0,
            limit_max_deceleration_mps2=2.5,
            limit_max_deceleration_jerk_mps3=1.5,
        )
        previous_positions = None
        previous_velocities = None
        previous_accelerations = None
        planned_ego_velocities = []
        planned_ego_accelerations = []
        stop_position = 6.0

        for cycle in range(18):
            ego_position = cycle * 0.27
            stations = []
            station = ego_position
            while station < 8.0 - 1.0e-6:
                stations.append(round(station, 8))
                station += 0.5
            stations.append(stop_position)
            stations = sorted(set(stations))
            positions = [(station, 0.0, 0.0) for station in stations]
            caps = [2.78 if station < stop_position else 0.0 for station in stations]

            previous_velocity = None
            previous_acceleration = None
            if previous_positions is not None:
                previous_state = project_motion_state(
                    previous_positions,
                    previous_velocities,
                    previous_accelerations,
                    (ego_position, 0.0, 0.0),
                )
                previous_velocity = previous_state.velocity_mps
                previous_acceleration = previous_state.acceleration_mps2
            initial_state = select_initial_state(
                2.78 if previous_velocity is None else previous_velocity,
                previous_velocity,
                previous_acceleration,
                0.5,
            )
            resampled = resample_profile_inputs(positions, caps, 0.1)
            profile = smooth_velocity_profile(
                resampled.positions,
                resampled.velocity_caps,
                initial_state.velocity_mps,
                initial_state.acceleration_mps2,
                parameters,
            )
            velocities = interpolate_values_at_stations(
                resampled.stations,
                profile.velocities,
                resampled.input_stations,
            )
            accelerations = interpolate_values_at_stations(
                resampled.stations,
                profile.accelerations,
                resampled.input_stations,
            )
            planned_ego_velocities.append(velocities[0])
            planned_ego_accelerations.append(accelerations[0])
            previous_positions = positions
            previous_velocities = velocities
            previous_accelerations = accelerations

        self.assertTrue(
            all(
                after <= before + 1.0e-6
                for before, after in zip(
                    planned_ego_velocities[1:-1],
                    planned_ego_velocities[2:],
                )
            )
        )
        self.assertTrue(
            all(acceleration <= 1.0e-6 for acceleration in planned_ego_accelerations[2:])
        )
        self.assertLess(planned_ego_velocities[-1], 1.5)

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

    def test_lateral_acceleration_limit_keeps_straight_caps(self):
        positions = straight_positions(10)
        caps = [10.0] * len(positions)
        output, count = apply_lateral_acceleration_limits(
            positions,
            caps,
            1.5,
            1.0,
            8.0,
            4.0,
            1,
            1.0,
            0.01,
        )
        self.assertEqual(count, 0)
        self.assertEqual(output, tuple(caps))

    def test_lateral_acceleration_limit_caps_curve_speed(self):
        positions = arc_positions(10.0, 11)
        caps = [10.0] * len(positions)
        output, count = apply_lateral_acceleration_limits(
            positions,
            caps,
            1.0,
            1.0,
            2.0,
            2.0,
            1,
            1.0,
            0.01,
        )
        self.assertGreater(count, 0)
        self.assertLess(min(output), 3.5)
        self.assertGreaterEqual(min(value for value in output if value > 0.01), 1.0)

    def test_lateral_acceleration_limit_preserves_stop_caps(self):
        positions = arc_positions(10.0, 11)
        caps = [8.0] * len(positions)
        caps[5] = 0.0
        output, count = apply_lateral_acceleration_limits(
            positions,
            caps,
            1.0,
            1.0,
            2.0,
            2.0,
            1,
            1.0,
            0.01,
        )
        self.assertGreater(count, 0)
        self.assertEqual(output[5], 0.0)

    def test_lateral_acceleration_limit_skips_infeasible_close_curve(self):
        positions = arc_positions(10.0, 11)
        caps = [10.0] * len(positions)
        output, count = apply_lateral_acceleration_limits(
            positions,
            caps,
            1.0,
            1.0,
            0.5,
            0.5,
            1,
            1.0,
            0.01,
            2.5,
            1.5,
            10.0,
        )
        self.assertEqual(count, 0)
        self.assertEqual(output, tuple(caps))

    def test_lateral_acceleration_limit_applies_when_deceleration_is_feasible(self):
        positions = straight_then_arc_positions(10.0, 11)
        caps = [10.0] * len(positions)
        output, count = apply_lateral_acceleration_limits(
            positions,
            caps,
            1.0,
            1.0,
            30.0,
            4.0,
            1,
            1.0,
            0.01,
            2.5,
            1.5,
            10.0,
        )
        self.assertGreater(count, 0)
        self.assertLess(min(output), 3.5)


if __name__ == "__main__":
    unittest.main()
