#!/usr/bin/env python3

import math
import importlib.util
import os
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_path_optimizer.mpt_optimizer import MPTParameters, ModelPredictiveTrajectory
from autoware_path_optimizer.trajectory_utils import TrajectorySample, curvature


def load_path_optimizer_node_module():
    script_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "scripts", "path_optimizer.py"
    )
    spec = importlib.util.spec_from_file_location("path_optimizer_node", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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

    def test_warm_start_is_aligned_by_position_after_path_crop_moves(self):
        source = [
            TrajectorySample(x=float(index + 2), y=0.0)
            for index in range(100)
        ]
        left = [(float(index + 2), 2.0, 0.0) for index in range(100)]
        right = [(float(index + 2), -2.0, 0.0) for index in range(100)]
        optimizer = ModelPredictiveTrajectory()
        optimizer.previous_points = [
            TrajectorySample(x=float(index), y=0.4)
            for index in range(110)
        ]

        result = optimizer.optimize(source, 7.0, 0.0, left, right)

        self.assertTrue(result.success, result.status)
        ego_index = min(
            range(len(result.points)),
            key=lambda index: abs(result.points[index].x - 7.0),
        )
        self.assertAlmostEqual(result.points[ego_index].y, 0.4, delta=0.02)

    def test_temporal_consistency_decays_before_far_return_path(self):
        source = path(count=100)
        left, right = bounds(count=100)
        optimizer = ModelPredictiveTrajectory(
            MPTParameters(
                temporal_consistency_decay_start=2.0,
                temporal_consistency_decay_end=10.0,
            )
        )
        optimizer.previous_points = [
            TrajectorySample(x=float(index), y=0.4)
            for index in range(100)
        ]

        result = optimizer.optimize(source, 5.0, 0.0, left, right)

        self.assertTrue(result.success, result.status)
        ego_index = min(
            range(len(result.points)),
            key=lambda index: abs(result.points[index].x - 5.0),
        )
        far_index = min(
            range(len(result.points)),
            key=lambda index: abs(result.points[index].x - 20.0),
        )
        self.assertAlmostEqual(result.points[ego_index].y, 0.4, delta=0.02)
        self.assertLess(abs(result.points[far_index].y), 0.1)

    def test_trajectory_over_steering_limit_is_rejected(self):
        source = path(lateral_noise=0.08)
        left, right = bounds()
        optimizer = ModelPredictiveTrajectory(
            MPTParameters(max_steer_angle=0.01)
        )

        result = optimizer.optimize(source, 2.0, source[2].y, left, right)

        self.assertFalse(result.success)
        self.assertEqual(result.status, "kinematic validation failure")

    def test_lateral_validation_failure_returns_fixed_points_cleanly(self):
        source = path()
        left = [(float(index), 3.0, 0.0) for index in range(len(source))]
        right = [(float(index), 0.5, 0.0) for index in range(len(source))]
        optimizer = ModelPredictiveTrajectory(
            MPTParameters(
                vehicle_width=1.0,
                soft_clearance_from_road=0.1,
                max_validation_lat_error=0.5,
            )
        )

        result = optimizer.optimize(source, 2.0, 0.0, left, right)

        self.assertFalse(result.success)
        self.assertEqual(result.status, "lateral validation failure")
        self.assertGreater(len(result.fixed_points), 0)

    def test_solver_failure_returns_fixed_points_cleanly(self):
        source = path()
        left, right = bounds()
        optimizer = ModelPredictiveTrajectory()

        with patch(
            "autoware_path_optimizer.mpt_optimizer.minimize",
            return_value=SimpleNamespace(
                success=False,
                x=[],
                message="forced failure",
            ),
        ):
            result = optimizer.optimize(source, 2.0, 0.0, left, right)

        self.assertFalse(result.success)
        self.assertEqual(result.status, "solver failure: forced failure")
        self.assertGreater(len(result.fixed_points), 0)

    def test_fallback_uses_current_input_instead_of_stale_previous_output(self):
        module = load_path_optimizer_node_module()
        node = module.PathOptimizerNode.__new__(module.PathOptimizerNode)
        node.output_interval = 0.5
        node.previous_output = [
            TrajectorySample(x=float(index - 30), y=0.0)
            for index in range(80)
        ]
        node.stop_before_outside = (
            lambda points, _left, _right, _ego_position=None: points
        )
        source = path(count=80)

        output = node.fallback(source, [], [])

        self.assertAlmostEqual(output[0].x, source[0].x)
        self.assertAlmostEqual(output[-1].x, source[-1].x)

    def test_fallback_ignores_drivable_area_violation_behind_ego(self):
        module = load_path_optimizer_node_module()
        node = module.PathOptimizerNode.__new__(module.PathOptimizerNode)
        node.enable_outside_stop = True
        node.stop_margin = 1.0
        node.output_interval = 0.5
        node.optimizer = SimpleNamespace(
            lateral_bounds=lambda points, _left, _right: (
                [0.2 if point.x < 3.0 else -1.0 for point in points],
                [1.0 for _point in points],
            )
        )
        source = path(count=20)

        output = node.fallback(source, [], [], ego_position=(5.0, 0.0))

        ego_index = min(
            range(len(output)), key=lambda index: abs(output[index].x - 5.0)
        )
        self.assertGreater(output[ego_index].longitudinal_velocity_mps, 0.0)

    def test_fallback_stops_for_drivable_area_violation_ahead_of_ego(self):
        module = load_path_optimizer_node_module()
        node = module.PathOptimizerNode.__new__(module.PathOptimizerNode)
        node.enable_outside_stop = True
        node.stop_margin = 1.0
        node.output_interval = 0.5
        node.optimizer = SimpleNamespace(
            lateral_bounds=lambda points, _left, _right: (
                [0.2 if point.x >= 8.0 else -1.0 for point in points],
                [1.0 for _point in points],
            )
        )
        source = path(count=20)

        output = node.fallback(source, [], [], ego_position=(5.0, 0.0))

        last_moving = max(
            point.x
            for point in output
            if point.longitudinal_velocity_mps > 0.0
        )
        self.assertLess(last_moving, 8.0)
        self.assertGreaterEqual(last_moving, 6.0)


if __name__ == "__main__":
    unittest.main()
