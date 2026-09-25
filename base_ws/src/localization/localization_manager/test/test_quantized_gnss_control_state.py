#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import sys
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "quantized_gnss_control_state_publisher.py"
)
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("quantized_control_state", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class QuantizedGnssControlStateTest(unittest.TestCase):
    def test_inside_cell_has_no_correction(self):
        correction = MODULE.quantized_cell_correction(
            1.04, 1.05, 1.0, 1.0, 0.1, 0.1, 0.0
        )
        self.assertAlmostEqual(correction[0], 0.0)
        self.assertAlmostEqual(correction[1], 0.0)

    def test_outside_cell_gets_minimum_correction(self):
        correction = MODULE.quantized_cell_correction(
            1.25, 0.7, 1.0, 1.0, 0.1, 0.2, 0.0
        )
        self.assertAlmostEqual(correction[0], -0.15)
        self.assertAlmostEqual(correction[1], 0.1)

    def test_rotated_cell_correction_is_frame_consistent(self):
        correction = MODULE.quantized_cell_correction(
            0.0, 0.25, 0.0, 0.0, 0.1, 0.2, math.pi / 2.0
        )
        self.assertAlmostEqual(correction[0], 0.0, places=9)
        self.assertAlmostEqual(correction[1], -0.15, places=9)

    def test_global_updates_only_yaw_without_discontinuity(self):
        alignment = MODULE.QuantizedGnssAlignment(
            0.1, 0.1, yaw_tau_sec=1.0, reset_translation_error_m=0.0
        )
        alignment.reset((2.0, 3.0, 0.0), 0.0, "initialization")
        value = alignment.update_global((5.0, 7.0, 0.2), (0.0, 0.0, 0.0), 0.1)
        self.assertAlmostEqual(value[0], 2.0)
        self.assertAlmostEqual(value[1], 3.0)
        self.assertGreater(value[2], 0.0)

    def test_yaw_update_preserves_vehicle_position_far_from_origin(self):
        alignment = MODULE.QuantizedGnssAlignment(
            0.1, 0.1, yaw_tau_sec=0.0, reset_translation_error_m=0.0
        )
        local_pose = (500.0, 300.0, 0.4)
        alignment.reset((2.0, 3.0, 0.0), 0.0, "initialization")
        before = MODULE.compose_planar(alignment.value, local_pose)
        alignment.update_global((0.0, 0.0, 0.1), local_pose, 0.1)
        after = MODULE.compose_planar(alignment.value, local_pose)
        self.assertAlmostEqual(before[0], after[0], places=9)
        self.assertAlmostEqual(before[1], after[1], places=9)
        self.assertAlmostEqual(after[2], local_pose[2] + 0.1, places=9)

    def test_cell_constraint_updates_alignment_translation(self):
        alignment = MODULE.QuantizedGnssAlignment(0.1, 0.2)
        alignment.reset((0.0, 0.0, 0.0), 0.0, "initialization")
        correction = alignment.constrain_position((1.5, 2.5, 0.0), 1.0, 2.0)
        self.assertAlmostEqual(correction[0], -0.4)
        self.assertAlmostEqual(correction[1], -0.3)
        pose = MODULE.compose_planar(alignment.value, (1.5, 2.5, 0.0))
        self.assertAlmostEqual(pose[0], 1.1)
        self.assertAlmostEqual(pose[1], 2.2)

    def test_checkpoint_global_jump_resets_candidate_to_same_pose(self):
        alignment = MODULE.QuantizedGnssAlignment(
            0.1,
            0.1,
            yaw_tau_sec=0.0,
            reset_translation_error_m=0.5,
        )
        local_pose = (100.0, 50.0, 0.2)
        initial_global = (102.0, 53.0, 0.2)
        alignment.reset(
            MODULE.planar_alignment(initial_global, local_pose),
            0.0,
            "initialization",
        )
        relocated_global = (102.0, 73.0, 0.2)
        target = MODULE.planar_alignment(relocated_global, local_pose)
        alignment.update_global(target, local_pose, 0.1)
        output = MODULE.compose_planar(alignment.value, local_pose)
        self.assertEqual(alignment.last_reset_reason, "translation_discontinuity")
        self.assertAlmostEqual(output[0], relocated_global[0], places=9)
        self.assertAlmostEqual(output[1], relocated_global[1], places=9)
        self.assertAlmostEqual(output[2], relocated_global[2], places=9)

    def test_blend_preserves_global_longitudinal_and_candidate_lateral(self):
        global_pose = (10.0, 5.0, math.radians(30.0))
        candidate_pose = (10.4, 4.7, math.radians(31.0))
        output = MODULE.preserve_global_longitudinal(global_pose, candidate_pose)
        sine = math.sin(global_pose[2])
        cosine = math.cos(global_pose[2])
        candidate_dx = candidate_pose[0] - global_pose[0]
        candidate_dy = candidate_pose[1] - global_pose[1]
        output_dx = output[0] - global_pose[0]
        output_dy = output[1] - global_pose[1]
        candidate_lateral = -candidate_dx * sine + candidate_dy * cosine
        output_lateral = -output_dx * sine + output_dy * cosine
        output_longitudinal = output_dx * cosine + output_dy * sine
        self.assertAlmostEqual(output_lateral, candidate_lateral, places=9)
        self.assertAlmostEqual(output_longitudinal, 0.0, places=9)
        self.assertAlmostEqual(output[2], candidate_pose[2])

    def test_planar_distance_is_frame_independent(self):
        self.assertAlmostEqual(MODULE.planar_distance(2.0, 3.0, -1.0, 7.0), 5.0)

    def test_measurement_freshness_uses_absolute_age(self):
        self.assertTrue(MODULE.measurement_is_fresh(10.0, 10.1, 0.2))
        self.assertTrue(MODULE.measurement_is_fresh(10.1, 10.0, 0.2))
        self.assertFalse(MODULE.measurement_is_fresh(10.0, 10.3, 0.2))
        self.assertFalse(MODULE.measurement_is_fresh(None, 10.0, 0.2))


if __name__ == "__main__":
    unittest.main()
