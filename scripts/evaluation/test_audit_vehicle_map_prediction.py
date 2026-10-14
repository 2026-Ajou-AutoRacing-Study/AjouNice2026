#!/usr/bin/env python3
"""Offline adapter and paired-cohort tests; no ROS master needed."""
import math
import unittest

from audit_vehicle_map_prediction import compare_paired, interpolate, normalize_frames


CANONICAL = dict(enabled=True, minimum_speed=1., enter=math.radians(100.), exit=math.radians(80.))


def core_track(velocity=(2., 0.), yaw=0.):
    return dict(id=3, core_class=1, confirmed=True, center=[1., 2., 3.],
                velocity=list(velocity), yaw=yaw, yaw_rate=.2)


class VehicleMapAuditTest(unittest.TestCase):
    def test_pi_canonicalization_preserves_world_velocity(self):
        track = core_track(velocity=(-2., 0.))
        row = normalize_frames([dict(stamp=100., tracks=[track])], "core", CANONICAL)[0]["objects"][0]
        self.assertAlmostEqual(abs(row["yaw"]), math.pi)
        self.assertEqual(row["world_velocity"], [-2., 0.])
        self.assertAlmostEqual(row["body_velocity"][0], 2.)
        self.assertAlmostEqual(row["body_velocity"][1], 0.)
        self.assertEqual(row["yaw_rate"], .2)

    def test_below_minimum_speed_retains_hysteresis_not_velocity(self):
        frames = [dict(stamp=100., tracks=[core_track(velocity=(-2., 0.))]),
                  dict(stamp=100.1, tracks=[core_track(velocity=(.2, 0.))]),
                  dict(stamp=100.2, tracks=[core_track(velocity=(2., 0.))])]
        rows = [r["objects"][0] for r in normalize_frames(frames, "core", CANONICAL)]
        self.assertAlmostEqual(abs(rows[1]["yaw"]), math.pi)
        self.assertAlmostEqual(rows[1]["body_velocity"][0], -.2)
        self.assertEqual(rows[1]["world_velocity"], [.2, 0.])
        self.assertAlmostEqual(rows[2]["yaw"], 0.)

    def test_ros_mode_world_velocity_is_derived_from_body_frame(self):
        obj = dict(id="abc", class_name="vehicle", center=[0., 0., 0.], yaw=math.pi/2,
                   body_velocity=[2., 1.], yaw_rate=0.)
        row = normalize_frames([dict(stamp=100., objects=[obj])], "ros", CANONICAL)[0]["objects"][0]
        self.assertAlmostEqual(row["world_velocity"][0], -1.)
        self.assertAlmostEqual(row["world_velocity"][1], 2.)

    def test_missing_yaw_rate_rejects_inexact_fallback_conversion(self):
        track = core_track()
        del track["yaw_rate"]
        with self.assertRaises(KeyError):
            normalize_frames([dict(stamp=100., tracks=[track])], "core", CANONICAL)

    def test_interpolation_refuses_large_gap_or_extrapolation(self):
        series = ([1., 1.1, 2.], [[0., 0.], [1., 0.], [2., 0.]])
        self.assertAlmostEqual(interpolate(series, 1.05)[0], .5)
        self.assertIsNone(interpolate(series, 1.5))
        self.assertIsNone(interpolate(series, .9))
        self.assertIsNone(interpolate(series, 2.1))

    def test_paired_gt_cohort_does_not_require_same_uuid(self):
        base = [dict(stamp=100., id="old", gt_key=["case", [38, 4]], horizon=3, top1_m=10.)]
        candidate = [dict(stamp=100., id="new", gt_key=["case", [38, 4]], horizon=3, top1_m=2.),
                     dict(stamp=100.1, id="new", gt_key=["case", [38, 4]], horizon=3, top1_m=3.)]
        report = compare_paired(base, candidate)
        self.assertEqual(report["3"]["paired"], 1)
        self.assertEqual(report["3"]["delta_candidate_minus_baseline_m"]["mean"], -8.)
        self.assertEqual(report["candidate_only"], 1)
        self.assertEqual(report["baseline_only"], 0)


if __name__ == "__main__":
    unittest.main()
