#!/usr/bin/env python3
"""Vehicle-pivot smoothing invariants, with no ROS master or controller writes."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "smooth_control_state_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("smooth_vehicle_pivot_contract", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SmoothVehiclePivotContractTest(unittest.TestCase):
    def assertPoseAlmostEqual(self, actual, expected, places=8):
        self.assertAlmostEqual(actual[0], expected[0], places=places)
        self.assertAlmostEqual(actual[1], expected[1], places=places)
        self.assertAlmostEqual(MODULE.normalize_angle(actual[2] - expected[2]), 0.0, places=places)

    @staticmethod
    def seeded_smoother(local, map_pose, **kwargs):
        smoother = MODULE.PlanarAlignmentSmoother(0.5, 0.7, **kwargs)
        smoother.reset(MODULE.planar_alignment(map_pose, local), 10.0)
        return smoother

    def test_output_is_invariant_to_odom_origin_translation_and_rotation(self):
        initial_local = (800.0, -600.0, 0.6)
        initial_global = (60.0, -10.0, 0.2)
        coordinate_changes = (
            (0.0, 0.0, 0.0),
            (16000.0, -24000.0, 0.0),
            (0.0, 0.0, 1.2),
            (-1000.0, 500.0, -2.9),
        )
        smoothers = [
            self.seeded_smoother(
                MODULE.compose_planar(change, initial_local),
                initial_global,
                lateral_translation_tau_sec=0.15,
                use_vehicle_pivot=True,
            )
            for change in coordinate_changes
        ]
        for index in range(1, 21):
            local = (800.0 + 0.8 * index, -600.0 + 0.1 * index, 0.6 + 0.025 * index)
            target_global = (60.0 + 0.5 * index, -10.0 + 0.4 * math.sin(index / 5.0), 0.2 + 0.04 * index)
            outputs = []
            for change, smoother in zip(coordinate_changes, smoothers):
                changed_local = MODULE.compose_planar(change, local)
                target_alignment = MODULE.planar_alignment(target_global, changed_local)
                value = smoother.update(target_alignment, 10.0 + index * 0.02, reference_pose=changed_local)
                outputs.append(MODULE.compose_planar(value, changed_local))
            for output in outputs[1:]:
                self.assertPoseAlmostEqual(output, outputs[0])

    def test_far_origin_yaw_only_correction_does_not_translate_vehicle(self):
        local = (500.0, 300.0, 0.0)
        old_pose = (100.0, 50.0, 0.0)
        target_pose = (100.0, 50.0, math.radians(10.0))
        smoother = self.seeded_smoother(local, old_pose, use_vehicle_pivot=True)
        value = smoother.update(MODULE.planar_alignment(target_pose, local), 10.02, reference_pose=local)
        output = MODULE.compose_planar(value, local)
        expected_yaw = MODULE.exponential_alpha(0.02, 0.7) * target_pose[2]
        self.assertPoseAlmostEqual(output, (100.0, 50.0, expected_yaw))

    def test_continuous_vehicle_motion_is_not_low_pass_filtered(self):
        alignment = (150.0, -75.0, -0.3)
        smoother = MODULE.PlanarAlignmentSmoother(
            0.5, 0.7, lateral_translation_tau_sec=0.15, use_vehicle_pivot=True
        )
        smoother.reset(alignment, 10.0)
        for index in range(1, 31):
            local = (800.0 + index, 300.0 + math.sin(index / 10.0), 0.2 + 0.01 * index)
            result = smoother.update(alignment, 10.0 + 0.02 * index, reference_pose=local)
            self.assertPoseAlmostEqual(
                MODULE.compose_planar(result, local), MODULE.compose_planar(alignment, local)
            )

    def test_yaw_rate_limit_does_not_move_vehicle_around_distant_origin(self):
        local = (1000.0, -300.0, 0.3)
        old_pose = (10.0, 20.0, 0.1)
        smoother = MODULE.PlanarAlignmentSmoother(0.0, 0.0, max_yaw_rate_radps=0.2, use_vehicle_pivot=True)
        smoother.reset(MODULE.planar_alignment(old_pose, local), 10.0)
        target = MODULE.planar_alignment((10.0, 20.0, 0.2), local)
        result = smoother.update(target, 10.1, reference_pose=local)
        self.assertPoseAlmostEqual(MODULE.compose_planar(result, local), (10.0, 20.0, 0.12))

    def test_legacy_mode_is_explicit_and_retains_alignment_space_behavior(self):
        local = (500.0, 300.0, 0.0)
        old_pose = (100.0, 50.0, 0.0)
        target = MODULE.planar_alignment((100.0, 50.0, math.radians(10.0)), local)
        initial = MODULE.planar_alignment(old_pose, local)
        smoother = self.seeded_smoother(local, old_pose, use_vehicle_pivot=False)
        value = smoother.update(target, 10.02, reference_pose=local)
        alpha = MODULE.exponential_alpha(0.02, 0.5)
        alpha_yaw = MODULE.exponential_alpha(0.02, 0.7)
        expected = (
            initial[0] + alpha * (target[0] - initial[0]),
            initial[1] + alpha * (target[1] - initial[1]),
            MODULE.normalize_angle(initial[2] + alpha_yaw * MODULE.normalize_angle(target[2] - initial[2])),
        )
        self.assertPoseAlmostEqual(value, expected)
        output = MODULE.compose_planar(value, local)
        self.assertGreater(math.hypot(output[0] - old_pose[0], output[1] - old_pose[1]), 0.1)

    def test_absent_reference_retains_legacy_alignment_smoothing(self):
        smoother = MODULE.PlanarAlignmentSmoother(0.5, 0.7, lateral_translation_tau_sec=0.1, use_vehicle_pivot=True)
        smoother.reset((5.0, 6.0, 0.2), 10.0)
        value = smoother.update((7.0, 5.0, 0.4), 10.1)
        alpha = MODULE.exponential_alpha(0.1, 0.5)
        expected = (5.0 + 2.0 * alpha, 6.0 - alpha, 0.2 + 0.2 * MODULE.exponential_alpha(0.1, 0.7))
        self.assertPoseAlmostEqual(value, expected)

    def test_longitudinal_and_lateral_alphas_use_target_vehicle_heading(self):
        local = (800.0, -300.0, -0.2)
        old_pose = (10.0, 20.0, 0.2)
        target_pose = (11.4, 19.2, math.pi / 3.0)
        smoother = self.seeded_smoother(local, old_pose, lateral_translation_tau_sec=0.1, use_vehicle_pivot=True)
        value = smoother.update(MODULE.planar_alignment(target_pose, local), 10.1, reference_pose=local)
        dx, dy = target_pose[0] - old_pose[0], target_pose[1] - old_pose[1]
        cosine, sine = math.cos(target_pose[2]), math.sin(target_pose[2])
        longitudinal = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        long_step = MODULE.exponential_alpha(0.1, 0.5) * longitudinal
        lateral_step = MODULE.exponential_alpha(0.1, 0.1) * lateral
        expected = (
            old_pose[0] + cosine * long_step - sine * lateral_step,
            old_pose[1] + sine * long_step + cosine * lateral_step,
            old_pose[2] + MODULE.exponential_alpha(0.1, 0.7) * (target_pose[2] - old_pose[2]),
        )
        self.assertPoseAlmostEqual(MODULE.compose_planar(value, local), expected)

    def test_unspecified_lateral_tau_matches_translation_tau(self):
        local = (1200.0, 300.0, 0.3)
        old_pose = (10.0, 20.0, 0.1)
        target = MODULE.planar_alignment((10.2, 20.3, 0.12), local)
        first = self.seeded_smoother(local, old_pose, use_vehicle_pivot=True)
        second = self.seeded_smoother(local, old_pose, use_vehicle_pivot=True, lateral_translation_tau_sec=0.5)
        self.assertPoseAlmostEqual(
            first.update(target, 10.02, reference_pose=local),
            second.update(target, 10.02, reference_pose=local),
        )

    def test_lateral_rate_cap_then_vector_rate_cap_apply_to_vehicle_correction(self):
        local = (500.0, -900.0, 0.4)
        old_pose = (10.0, 20.0, 0.0)
        target = MODULE.planar_alignment((11.0, 21.0, 0.0), local)
        smoother = MODULE.PlanarAlignmentSmoother(
            0.0,
            0.0,
            max_translation_rate_mps=0.5,
            lateral_translation_tau_sec=0.0,
            use_vehicle_pivot=True,
            max_lateral_translation_rate_mps=0.2,
        )
        smoother.reset(MODULE.planar_alignment(old_pose, local), 10.0)
        value = smoother.update(target, 10.1, reference_pose=local)
        output = MODULE.compose_planar(value, local)
        # First the body-lateral step is limited to 0.02 m; then the total
        # (1, 0.02) m vector is limited to 0.05 m without changing direction.
        expected_step = MODULE.limit_vector_step(1.0, 0.02, 0.05)
        self.assertPoseAlmostEqual(output, (10.0 + expected_step[0], 20.0 + expected_step[1], 0.0))
        self.assertLessEqual(math.hypot(output[0] - 10.0, output[1] - 20.0), 0.050000001)

    def test_invalid_input_rejected_before_initialization_without_poisoning(self):
        for target, stamp, reference in (
            ((float("nan"), 0.0, 0.0), 10.0, (0.0, 0.0, 0.0)),
            ((0.0, float("inf"), 0.0), 10.0, (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), float("nan"), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), 10.0, (0.0, 0.0, float("nan"))),
        ):
            with self.subTest(target=target, stamp=stamp, reference=reference):
                smoother = MODULE.PlanarAlignmentSmoother(0.5, 0.5, use_vehicle_pivot=True)
                with self.assertRaises(ValueError):
                    smoother.update(target, stamp, reference_pose=reference)
                self.assertIsNone(smoother.value)
                self.assertIsNone(smoother.stamp_sec)
                self.assertPoseAlmostEqual(smoother.update((0.0, 0.0, 0.0), 10.0), (0.0, 0.0, 0.0))

    def test_invalid_updates_and_duplicate_stamps_do_not_poison_prior_state(self):
        local = (500.0, 300.0, 0.0)
        smoother = self.seeded_smoother(local, (100.0, 50.0, 0.0), use_vehicle_pivot=True)
        original, original_stamp = smoother.value, smoother.stamp_sec
        valid = MODULE.planar_alignment((100.05, 50.0, 0.0), local)
        for target, stamp, reference in (
            ((0.0, 0.0, float("inf")), 10.02, local),
            (valid, float("nan"), local),
            (valid, 10.02, (float("nan"), 0.0, 0.0)),
            (valid, 10.0, local),
            (valid, 9.99, local),
        ):
            with self.subTest(target=target, stamp=stamp, reference=reference):
                with self.assertRaises(ValueError):
                    smoother.update(target, stamp, reference_pose=reference)
                self.assertEqual(smoother.value, original)
                self.assertEqual(smoother.stamp_sec, original_stamp)
        smoother.update(valid, 10.02, reference_pose=local)
        self.assertEqual(smoother.stamp_sec, 10.02)

    def test_far_origin_normal_correction_and_checkpoint_use_vehicle_pose_guard(self):
        local = (500.0, 300.0, 0.0)
        initial_pose = (100.0, 50.0, 0.0)
        smoother = self.seeded_smoother(local, initial_pose, reset_translation_error_m=0.5, use_vehicle_pivot=True)
        small_target = MODULE.planar_alignment((100.05, 50.0, math.radians(0.1)), local)
        smoother.update(small_target, 10.02, reference_pose=local)
        self.assertIsNone(smoother.last_reset_reason)
        checkpoint_target = MODULE.planar_alignment((120.0, 50.0, math.radians(0.1)), local)
        value = smoother.update(checkpoint_target, 10.04, reference_pose=local)
        self.assertEqual(value, checkpoint_target)
        self.assertEqual(smoother.last_reset_reason, "translation_discontinuity")

    def test_yaw_wrap_uses_short_arc_at_far_origin(self):
        local = (1000.0, -600.0, 0.2)
        old_pose = (10.0, 20.0, math.radians(179.0))
        target_pose = (10.0, 20.0, math.radians(-179.0))
        smoother = self.seeded_smoother(local, old_pose, use_vehicle_pivot=True)
        value = smoother.update(MODULE.planar_alignment(target_pose, local), 10.1, reference_pose=local)
        expected_yaw = MODULE.normalize_angle(old_pose[2] + MODULE.exponential_alpha(0.1, 0.7) * math.radians(2.0))
        self.assertPoseAlmostEqual(MODULE.compose_planar(value, local), (10.0, 20.0, expected_yaw))

    def test_new_parameter_validation_is_strict(self):
        for kwargs in (
            {"lateral_translation_tau_sec": -0.1},
            {"lateral_translation_tau_sec": float("nan")},
            {"max_lateral_translation_rate_mps": -1.0},
            {"max_lateral_translation_rate_mps": float("inf")},
            {"use_vehicle_pivot": "false"},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    MODULE.PlanarAlignmentSmoother(0.5, 0.5, **kwargs)


if __name__ == "__main__":
    unittest.main()
