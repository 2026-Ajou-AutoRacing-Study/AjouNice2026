#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import generate_constant_curvature_return as generator


class ConstantCurvatureReturnTest(unittest.TestCase):
    def test_start_pose_uses_first_lanelet_point_and_forward_tangent(self):
        points = [
            (1.0, 2.0, 3.0),
            (1.0, 2.0, 3.0),
            (4.0, 6.0, 3.1),
            (8.0, 6.0, 3.2),
        ]
        with mock.patch.object(
            generator.base, "orient_centerline", return_value=points
        ):
            pose = generator.start_lanelet_start_pose(
                [{"lanelet_id": 10}], {10: points}, []
            )
        self.assertEqual(pose[:3], points[0])
        self.assertAlmostEqual(pose[3], math.atan2(4.0, 3.0))

    def test_filename_number_is_compact_and_stable(self):
        self.assertEqual(generator.filename_number(0.03), "0.03")
        self.assertEqual(generator.filename_number(7.5), "7.5")
        self.assertEqual(generator.filename_number(20.0), "20")
        self.assertEqual(generator.filename_token(0.03), "0p03")
        self.assertEqual(generator.filename_token(7.5), "7p5")

    def test_lane_return_matches_closed_form_endpoint(self):
        kappa = 0.03
        segment_length = 7.5
        result = generator.generate_lane_return(
            start_x_m=0.0,
            start_y_m=0.0,
            start_z_m=1.0,
            start_yaw_rad=0.0,
            kappa_mag=kappa,
            segment_length_m=segment_length,
            sample_interval_m=0.1,
            desired_velocity_mps=5.0,
            lanelet_id=200101,
            route_order=28,
        )

        expected_x = 4.0 * math.sin(kappa * segment_length) / kappa
        endpoint = result.points[-1]
        self.assertAlmostEqual(endpoint["x"], expected_x, places=9)
        self.assertAlmostEqual(endpoint["y"], 0.0, places=9)
        self.assertAlmostEqual(endpoint["yaw"], 0.0, places=12)
        self.assertEqual(endpoint["z"], 1.0)
        self.assertEqual(len(result.segment_ranges), 4)

    def test_segment_boundaries_are_not_duplicated(self):
        result = generator.generate_lane_return(
            0.0, 0.0, 0.0, 0.0, 0.03, 7.55, sample_interval_m=0.1
        )
        for first, second in zip(result.points, result.points[1:]):
            self.assertGreater(
                math.hypot(second["x"] - first["x"], second["y"] - first["y"]),
                0.0,
            )
        for previous, following in zip(
            result.segment_ranges, result.segment_ranges[1:]
        ):
            self.assertEqual(previous[1] + 1, following[0])

    def test_straights_are_inserted_before_and_between_arc_pairs(self):
        kappa = 0.03
        arc_length = 7.5
        start_straight = 4.0
        middle_straight = 6.0
        result = generator.generate_lane_return(
            0.0,
            0.0,
            0.0,
            0.0,
            kappa,
            arc_length,
            start_straight_length_m=start_straight,
            middle_straight_length_m=middle_straight,
        )

        self.assertEqual(
            result.metadata["segment_roles"],
            [
                "start_straight",
                "outbound_positive_arc",
                "outbound_negative_arc",
                "middle_straight",
                "return_negative_arc",
                "return_positive_arc",
            ],
        )
        middle_start = result.segment_ranges[3][0]
        middle_end = result.segment_ranges[3][1]
        self.assertAlmostEqual(result.points[middle_start]["yaw"], 0.0, places=12)
        self.assertAlmostEqual(result.points[middle_end]["yaw"], 0.0, places=12)
        expected_x = (
            start_straight
            + middle_straight
            + 4.0 * math.sin(kappa * arc_length) / kappa
        )
        self.assertAlmostEqual(result.points[-1]["x"], expected_x, places=9)
        self.assertAlmostEqual(result.points[-1]["y"], 0.0, places=9)
        self.assertAlmostEqual(
            result.metadata["total_length_m"],
            start_straight + middle_straight + 4.0 * arc_length,
        )

    def test_zero_curvature_is_straight_baseline(self):
        result = generator.generate_lane_return(
            1.0, 2.0, 3.0, math.pi / 2.0, 0.0, 2.0
        )
        endpoint = result.points[-1]
        self.assertAlmostEqual(endpoint["x"], 1.0, places=9)
        self.assertAlmostEqual(endpoint["y"], 10.0, places=9)
        self.assertIsNone(result.metadata["radius_m"])
        self.assertEqual(result.metadata["design_w_peak_m"], 0.0)
        self.assertEqual(
            {
                point["segment_sign"]
                for point in result.points
                if point["segment_id"] == 0
            },
            {1},
        )
        self.assertEqual(
            {
                point["segment_sign"]
                for point in result.points
                if point["segment_id"] == 1
            },
            {-1},
        )

    def test_rejects_non_monotonic_design(self):
        with self.assertRaisesRegex(ValueError, "delta_theta >= 90 deg"):
            generator.generate_lane_return(
                0.0, 0.0, 0.0, 0.0, math.pi / 2.0, 1.0
            )

    def test_high_curvature_uses_arc_chord_spacing(self):
        with self.assertWarns(UserWarning):
            result = generator.generate_lane_return(
                0.0, 0.0, 0.0, 0.0, 0.209, 7.5
            )
        self.assertEqual(len(result.points), 301)
        self.assertAlmostEqual(result.points[-1]["y"], 0.0, places=8)


if __name__ == "__main__":
    unittest.main()
