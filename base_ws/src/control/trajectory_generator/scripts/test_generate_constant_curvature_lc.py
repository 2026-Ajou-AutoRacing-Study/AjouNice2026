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

    def test_lane_change_starts_at_source_lanelet_end(self):
        source = [(0.0, 10.0, 0.0), (0.0, 5.0, 0.0), (0.0, 0.0, 0.0)]
        target = [(3.5, 0.0, 0.0), (3.5, -200.0, 0.0)]

        geometry = generator.build_lane_change_geometry(
            source, target, 0.0014
        )

        self.assertEqual(geometry["start"], source[-1])
        self.assertAlmostEqual(geometry["target_width_m"], 3.5)
        self.assertAlmostEqual(
            geometry["target_station"],
            2.0
            * math.sin(
                0.0014 * geometry["segments"][0].length_m
            )
            / 0.0014,
        )

    def test_repeated_lane_changes_use_straights_between_arc_pairs(self):
        geometry = generator.build_repeated_lane_change_segments(
            kappa_mag=0.03,
            lane_width_m=3.5,
            direction="left",
            lane_change_count=2,
            initial_straight_length_m=10.0,
            between_lane_change_length_m=5.0,
            final_straight_length_m=8.0,
        )
        self.assertEqual(
            geometry["segment_roles"],
            [
                "initial_straight",
                "lane_change_1_entry_arc",
                "lane_change_1_exit_arc",
                "between_lane_change_1_straight",
                "lane_change_2_entry_arc",
                "lane_change_2_exit_arc",
                "final_straight",
            ],
        )
        result = generator.curvature.generate_from_curvature_segments(
            0.0,
            0.0,
            0.0,
            0.0,
            geometry["segments"],
            0.1,
            10.0,
            1,
            0,
        )
        self.assertAlmostEqual(result.points[-1]["yaw"], 0.0, places=12)
        self.assertAlmostEqual(result.points[-1]["y"], 7.0, places=9)

    def test_total_length_is_filled_by_final_straight(self):
        final, minimum = generator.resolve_final_straight_length(
            total_length_m=200.0,
            kappa_mag=0.01,
            lane_width_m=3.5,
            lane_change_count=2,
            initial_straight_length_m=30.0,
            between_lane_change_length_m=10.0,
        )
        self.assertAlmostEqual(final + minimum, 200.0)
        self.assertGreater(final, 0.0)

    def test_rejects_total_length_shorter_than_maneuver(self):
        with self.assertRaisesRegex(ValueError, "require at least"):
            generator.resolve_final_straight_length(
                total_length_m=50.0,
                kappa_mag=0.01,
                lane_width_m=3.5,
                lane_change_count=2,
                initial_straight_length_m=30.0,
                between_lane_change_length_m=10.0,
            )


if __name__ == "__main__":
    unittest.main()
