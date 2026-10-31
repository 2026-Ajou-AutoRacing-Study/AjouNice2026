#!/usr/bin/env python3
"""Deterministic tests: no ROS graph, simulator, network or real vehicle."""
import importlib.util
import json
import math
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import analyze_gnss_delay as M


def fake_message_bag(messages):
    class FakeBag:
        def __init__(self, path):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def get_start_time(self):
            return 1000.

        def read_messages(self, topics):
            return iter(row for row in messages if row[0] in topics)
    return SimpleNamespace(Bag=FakeBag)


def stamp(t):
    seconds = int(math.floor(t))
    nanoseconds = int(round((t-seconds)*1e9))
    return SimpleNamespace(to_sec=lambda: t, secs=seconds, nsecs=nanoseconds)


def canonical_numeric(times, values):
    seconds = np.floor(times).astype(int)
    nanoseconds = np.rint((times-seconds)*1e9).astype(int)
    return np.column_stack([times, values, seconds+1000, nanoseconds, np.ones(len(times))])


def reader_messages(reverse_tf=False):
    yaw = .2
    offset = [4., 6.]
    if reverse_tf:
        offset = -M.transform_xy(offset, [0, 0, -yaw])
        yaw = -yaw
    q = SimpleNamespace(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))
    tf = SimpleNamespace(header=SimpleNamespace(frame_id="morai_map" if reverse_tf else "map"),
                         child_frame_id="map" if reverse_tf else "morai_map",
                         transform=SimpleNamespace(translation=SimpleNamespace(x=offset[0], y=offset[1]), rotation=q))
    rows = [("/tf_static", SimpleNamespace(transforms=[tf]), stamp(1000.))]
    for index in range(4):
        t = 1000.+.1*index
        lat, lon = 37.24+.000001*index, 127.09
        east, north = M.wgs84_to_utm(lat, lon)
        ego = SimpleNamespace(header=SimpleNamespace(stamp=stamp(t)), position=SimpleNamespace(x=east-302595., y=north-4124145.),
                              velocity=SimpleNamespace(x=10.), heading=1.347)
        gps = SimpleNamespace(header=SimpleNamespace(stamp=stamp(t)), latitude=lat, longitude=lon,
                              status=1, eastOffset=302595., northOffset=4124145.)
        rows.extend([("/Ego_topic", ego, stamp(t)), ("/sensor/gps", gps, stamp(t))])
    return rows


def synthetic(delay_ms=20., bias=0., constant_speed=False, duration=60., noise=.002):
    t = np.arange(0., duration+2., .02)
    if constant_speed:
        speed = np.full(len(t), 10.)
        x = t*10.
    else:
        speed = 10.+4.*np.sin(t*.24)
        x = 10.*t+4./.24*(1.-np.cos(t*.24))
    ego = np.column_stack([t, x, np.zeros(len(t)), np.zeros(len(t)), speed])
    gt = np.arange(1., duration, .05)
    past, _ = M.oracle_at(ego, gt-delay_ms/1000.)
    rng = np.random.RandomState(19)
    gps = np.column_stack([gt, past[:, 0]+bias+rng.normal(0, noise, len(gt)), past[:, 1]])
    return {"ego": ego, "gps": gps}


class DelayAnalyzerTests(unittest.TestCase):
    def test_known_twenty_ms_is_absolute_and_improves_same_samples(self):
        result = M.evaluate(synthetic())
        self.assertEqual(result["status"], "ok", result["reasons"])
        self.assertEqual(result["recommendation_ms"], 20)
        self.assertAlmostEqual(result["apparent_relative_lag_ms"], 20, delta=1.)
        comparison = result["evidence"]["input_shift_comparison"]
        self.assertLess(comparison["recommended"]["along_mae_m"], .01)
        self.assertGreater(comparison["baseline"]["along_mae_m"], .15)
        self.assertFalse(result["settings_applied"])
        self.assertFalse(result["physical_network_latency_measured"])
        self.assertEqual(result["recommendation_scope"], "absolute_raw_gnss_advance")

    def test_fixed_spatial_bias_is_not_called_time_delay(self):
        result = M.evaluate(synthetic(delay_ms=0, bias=-.35))
        self.assertEqual(result["status"], "ok", result["reasons"])
        self.assertEqual(result["recommendation_ms"], 0)
        self.assertAlmostEqual(result["spatial_bias_m"], -.35, delta=.005)
        self.assertAlmostEqual(result["apparent_relative_lag_ms"], 0, delta=1.)
        self.assertGreater(result["evidence"]["zero_spatial_bias_grid_best_ms"], 20)

    def test_spatial_bias_and_time_can_be_separated(self):
        result = M.evaluate(synthetic(delay_ms=20, bias=-.1))
        self.assertEqual(result["status"], "ok", result["reasons"])
        self.assertAlmostEqual(result["spatial_bias_m"], -.1, delta=.005)
        self.assertEqual(result["recommendation_ms"], 20)

    def test_constant_speed_is_not_identifiable(self):
        result = M.evaluate(synthetic(constant_speed=True))
        self.assertEqual(result["status"], "insufficient_evidence")
        self.assertIsNone(result["recommendation_ms"])
        self.assertTrue(any("not identifiable" in r for r in result["reasons"]))

    def test_short_run_and_low_speed_hold(self):
        result = M.evaluate(synthetic(duration=8))
        self.assertEqual(result["status"], "insufficient_evidence")
        data = synthetic()
        data["ego"][:, 4] = 1.
        result = M.evaluate(data)
        self.assertEqual(result["status"], "insufficient_evidence")

    def test_no_raw_or_no_ego_never_falls_back_to_filtered_position(self):
        for key in ("gps", "ego"):
            data = synthetic()
            del data[key]
            result = M.evaluate(data)
            self.assertEqual(result["status"], "insufficient_evidence")
            self.assertIsNone(result["recommendation_ms"])

    def test_no_interpolation_across_gap_or_teleport(self):
        ego = np.array([[0, 0, 0, 0, 10], [.1, 1, 0, 0, 10],
                        [1, 10, 0, 0, 10], [1.1, 100, 0, 0, 10], [1.2, 101, 0, 0, 10]])
        _, good = M.oracle_at(ego, [.05, .5, 1.05, 1.15])
        self.assertEqual(good.tolist(), [True, False, False, True])

    def test_gap_and_teleport_exclusion_preserve_correct_estimate(self):
        data = synthetic()
        ego = data["ego"]
        # A missing interval is excluded, and the same teleport is in both streams.
        data["ego"] = ego[(ego[:, 0] < 10) | (ego[:, 0] > 11)]
        data["ego"][data["ego"][:, 0] > 35, 1] += 100
        data["gps"][data["gps"][:, 0] > 35.02, 1] += 100
        result = M.evaluate(data)
        self.assertEqual(result["status"], "ok", result["reasons"])
        self.assertEqual(result["recommendation_ms"], 20)
        self.assertGreater(result["evidence"]["rejected_pair_samples"], 20)

    def test_clean_duplicate_out_of_order_nonfinite(self):
        rows = [[2, 2, 2], [1, 1, 1], [1, 9, 9], [3, np.nan, 3]]
        data, counts = M.clean_series(rows, 3)
        self.assertEqual(data.tolist(), [[1., 1., 1.], [2., 2., 2.]])
        self.assertEqual(counts, {"original": 4, "nonfinite": 1, "duplicates": 1, "backwards": 1,
                                  "large_backwards": 1})

    def test_overloaded_position_velocity_mismatch_is_not_delay_calibration(self):
        data = synthetic()
        data["ego"][:, 1] *= .75
        data["gps"][:, 1] *= .75
        result = M.evaluate(data)
        self.assertEqual(result["status"], "insufficient_evidence")
        self.assertTrue(any("wall-time" in reason for reason in result["reasons"]))

    def test_changed_epoch_between_time_halves_holds(self):
        data = synthetic()
        times = data["gps"][:, 0]
        target, _ = M.oracle_at(data["ego"], times-.07)
        data["gps"][times > 30, 1] = target[times > 30, 0]
        result = M.evaluate(data)
        self.assertEqual(result["status"], "insufficient_evidence")
        self.assertTrue(any("half" in r or "uncertainty" in r for r in result["reasons"]))

    def test_compensated_output_has_no_effect_on_absolute_raw_result(self):
        data = synthetic()
        data["gnss"] = np.ones((100, 7))*1e9
        result = M.evaluate(data, provenance={"recorded_configured_advance_ms": [20]})
        self.assertEqual(result["recommendation_ms"], 20)
        self.assertEqual(result["provenance"]["recorded_configured_advance_ms"], [20])

    def test_transform_and_heading_convention_once(self):
        mapped = M.transform_xy(np.array([[1., 0.]]), [4., 6., math.pi/2])
        np.testing.assert_allclose(mapped, [[4., 7.]])
        # Canonical orientation already map-frame; do not add MR offset again.
        data = synthetic()
        data["twist"] = canonical_numeric(data["ego"][:, 0], data["ego"][:, 4])
        data["orientation"] = canonical_numeric(data["ego"][:, 0], data["ego"][:, 3])
        result = M.evaluate(data)
        self.assertEqual(result["recommendation_ms"], 20)
        comparison = result["evidence"]["input_shift_comparison"]
        self.assertIn("canonical", comparison["motion_source"])
        self.assertEqual(comparison["diagnostic_header_pair_valid_samples"], comparison["sample_count"])
        self.assertLess(comparison["recommended"]["along_mae_m"], .02)
        self.assertFalse(comparison["actual_callback_applied_rate_measured"])

    def test_five_ms_mismatched_canonical_stamps_never_advance(self):
        data = synthetic()
        data["twist"] = canonical_numeric(data["ego"][:, 0]+.005, data["ego"][:, 4])
        data["orientation"] = canonical_numeric(data["ego"][:, 0], data["ego"][:, 3])
        # Even available Competition must not hide rejected canonical pairs.
        data["competition"] = data["ego"].copy()
        result = M.evaluate(data)
        comparison = result["evidence"]["input_shift_comparison"]
        self.assertEqual(comparison["diagnostic_header_pair_valid_samples"], 0)
        self.assertEqual(comparison["baseline"], comparison["recommended"])
        self.assertIn("invalid pairs unadvanced", comparison["motion_source"])

    def test_missing_canonical_pair_does_not_fall_back_to_oracle(self):
        data = synthetic()
        data["twist"] = canonical_numeric(data["ego"][:, 0], data["ego"][:, 4])
        result = M.evaluate(data)
        comparison = result["evidence"]["input_shift_comparison"]
        self.assertEqual(comparison["diagnostic_header_pair_valid_samples"], 0)
        self.assertEqual(comparison["baseline"], comparison["recommended"])

    def test_nanosecond_stamp_difference_is_not_lost_in_epoch_float(self):
        data = synthetic()
        data["twist"] = canonical_numeric(data["ego"][:, 0], data["ego"][:, 4])
        data["orientation"] = canonical_numeric(data["ego"][:, 0], data["ego"][:, 3])
        data["orientation"][:, 3] += 1.
        result = M.evaluate(data)
        comparison = result["evidence"]["input_shift_comparison"]
        self.assertEqual(comparison["diagnostic_header_pair_valid_samples"], 0)
        self.assertEqual(comparison["baseline"], comparison["recommended"])

    def test_canonical_reader_validation_keeps_invalid_latest_rows(self):
        header = SimpleNamespace(stamp=stamp(1000.005), frame_id="base_link")
        msg = SimpleNamespace(header=header, orientation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                              orientation_covariance=[0.]*9,
                              twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=12.))))
        self.assertEqual(M.canonical_motion_row(msg, "twist", .005), [.005, 12., 1000, 5000000, 1.])
        self.assertEqual(M.canonical_motion_row(msg, "orientation", .005), [.005, 0., 1000, 5000000, 1.])
        for invalid in ("frame", "unavailable", "quaternion"):
            msg.header.frame_id = "wrong_frame" if invalid == "frame" else "base_link"
            msg.orientation_covariance[0] = -1. if invalid == "unavailable" else 0.
            msg.orientation.w = 1.1 if invalid == "quaternion" else 1.
            row = M.canonical_motion_row(msg, "orientation", .005)
            self.assertEqual(row[-1], 0.)
            self.assertEqual(row[0], .005)

    def test_converter_equivalent_wgs84_function(self):
        # AST extracts only a pure existing function; no rospy or ROS initialization.
        import ast
        source = Path(__file__).resolve().parents[3]/"base_ws/src/localization/localization_manager/scripts/morai_gps_to_local_odometry.py"
        tree = ast.parse(source.read_text())
        selected = ast.Module(body=[node for node in tree.body if isinstance(node, ast.FunctionDef)
                                   and node.name == "wgs84_to_utm"], type_ignores=[])
        scope = {"math": math, "WGS84_A": 6378137., "WGS84_INV_F": 298.257223563, "UTM_SCALE": .9996}
        exec(compile(selected, str(source), "exec"), scope)
        for lat, lon in ((37.24, 127.09), (37.25, 127.12), (-33., 18.)):
            zone, north = (52, True) if lat > 0 else (34, False)
            np.testing.assert_allclose(M.wgs84_to_utm(lat, lon, zone, north),
                                       scope["wgs84_to_utm"](lat, lon, zone, north), rtol=0, atol=1e-8)

    def test_reader_applies_recorded_transform_and_heading_exactly_once(self):
        messages = reader_messages()
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(messages)}):
            data, provenance = M.read_bag("synthetic.bag", -1.347)
        np.testing.assert_allclose(data["ego"][:, 1:3], data["gps"][:, 1:3], atol=1e-8)
        np.testing.assert_allclose(data["ego"][:, 3], .2, atol=1e-12)
        self.assertEqual(provenance["heading_offset_deg_applied_once_to_raw_status"], -1.347)
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(reader_messages(reverse_tf=True))}):
            reverse, _ = M.read_bag("reverse.bag", -1.347)
        np.testing.assert_allclose(reverse["gps"], data["gps"], atol=1e-8)

    def test_reader_zero_gps_status_and_zero_headers_are_rejected(self):
        messages = reader_messages()
        messages[2][1].latitude = 0.
        messages[2][1].longitude = 0.
        messages[4][1].status = 0
        messages[6][1].header.stamp = stamp(0)
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(messages)}):
            data, provenance = M.read_bag("synthetic.bag")
        self.assertEqual(len(data["gps"]), 1)
        self.assertEqual(provenance["invalid_zero_or_status_gps_samples"], 2)
        self.assertIn("Rejected 1 zero", provenance["notices"][0])

    def test_reader_missing_or_changing_transform_is_insufficient(self):
        messages = reader_messages()
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(messages[1:])}):
            with self.assertRaises(M.InsufficientEvidence):
                M.read_bag("missing_tf.bag")
        changed = reader_messages(reverse_tf=True)[0]
        changed[1].transforms[0].transform.translation.x += 1.
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(messages+[changed])}):
            with self.assertRaises(M.InsufficientEvidence):
                M.read_bag("changing_tf.bag")

    def test_reader_offset_fallback_matches_production_threshold(self):
        messages = reader_messages()
        for topic, msg, receipt in messages:
            if topic == "/sensor/gps":
                msg.eastOffset = .5
                msg.northOffset = float("nan")
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(messages)}):
            data, _ = M.read_bag("offset_fallback.bag")
        np.testing.assert_allclose(data["ego"][:, 1:3], data["gps"][:, 1:3], atol=1e-8)

    def test_reader_clock_reset_does_not_sort_overlapping_epochs_into_fake_track(self):
        messages = reader_messages()
        messages[-1][1].header.stamp = stamp(999.)
        with patch.dict(sys.modules, {"rosbag": fake_message_bag(messages)}):
            with self.assertRaises(M.InsufficientEvidence):
                M.read_bag("clock_reset.bag")

    def test_multi_recording_consistency(self):
        a = M.evaluate(synthetic(20))
        b = M.evaluate(synthetic(25))
        self.assertEqual(M.aggregate([a, b])["status"], "ok")
        c = M.evaluate(synthetic(55))
        self.assertEqual(M.aggregate([a, c])["status"], "insufficient_evidence")

    def test_cli_insufficient_writes_result_and_report(self):
        with TemporaryDirectory() as directory, patch.object(M, "read_bag", side_effect=M.InsufficientEvidence("Missing recorded map/morai_map TF")):
            status = M.main(["missing_tf.bag", "--output-dir", directory])
            self.assertEqual(status, 2)
            result = json.loads((Path(directory)/"result.json").read_text())
            self.assertIsNone(result["recommendation_ms"])
            self.assertIn("Missing recorded", (Path(directory)/"report.txt").read_text())

    def test_cli_processing_error_is_not_success(self):
        with TemporaryDirectory() as directory, patch.object(M, "read_bag", side_effect=OSError("Broken bag")):
            self.assertEqual(M.main(["broken.bag", "--output-dir", directory]), 1)
            result = json.loads((Path(directory)/"result.json").read_text())
            self.assertEqual(result["status"], "error")
            self.assertIsNone(result["recommendation_ms"])


if __name__ == "__main__":
    unittest.main()
