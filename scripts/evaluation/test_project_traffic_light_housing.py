#!/usr/bin/env python3
"""Pure numerical/static-file tests: no ROS, models, simulator, or network."""

import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from project_traffic_light_housing import (
    fixed_camera, load_fixed_camera, load_signal, parse_ego_document,
    project_housing, project_point, world_to_camera, write_json,
)


def signal(center=(10., 0., 0.), heading=0., width=1., height=.5):
    return {"idx": "fixture", "point": list(center), "heading": heading,
            "width": width, "height": height, "z_offset": 4.5}


def zero_camera():
    camera = fixed_camera()
    camera.update(position_xyz_m=[0., 0., 0.], pitch_degrees=0.)
    return camera


class HousingProjectionTest(unittest.TestCase):
    def test_c16_reference_center_is_about_503_184(self):
        housing = signal(center=(-64.1, -25.36, 34.25341), heading=-89.678, width=1.42, height=.355)
        housing["idx"] = "C1256W000016"
        result = project_housing(housing, [-60.09483, -45.53425, 28.7261], 91.663)
        u, v = result["projected_center_uv"]
        self.assertAlmostEqual(u, 503, delta=2)
        self.assertAlmostEqual(v, 184, delta=2)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["intrinsics"]["fx"], 640)
        self.assertFalse(result["ground_truth"])
        self.assertTrue(result["requires_visual_verification"])

    def test_axis_sign_left_maps_left_and_up_maps_up(self):
        result = project_point([10, 1, 1], [0, 0, 0], 1.347, zero_camera())
        self.assertEqual(result["camera_xyz_forward_left_up_m"], [10, 1, 1])
        self.assertAlmostEqual(result["image_uv"][0], 576)
        self.assertAlmostEqual(result["image_uv"][1], 296)

    def test_yaw_correction_once_and_world_translation_equivalence(self):
        first = project_point([-1, 10, 0], [0, 0, 0], 91.347, zero_camera())
        second = project_point([99, 210, 30], [100, 200, 30], 91.347, zero_camera())
        for actual, expected in zip(first["image_uv"], [576, 360]):
            self.assertAlmostEqual(actual, expected)
        for left, right in zip(first["image_uv"], second["image_uv"]):
            self.assertAlmostEqual(left, right)

    def test_camera_translation_is_subtracted_in_ego_frame(self):
        camera = zero_camera()
        camera["position_xyz_m"] = [1.9, 0., 1.2]
        result = world_to_camera([10, 0, 2], [0, 0, 0], 1.347, camera)
        for actual, expected in zip(result["camera_xyz_forward_left_up_m"], [8.1, 0., .8]):
            self.assertAlmostEqual(actual, expected)

    def test_camera_pitch_sign_matches_explicit_contract(self):
        camera = zero_camera()
        camera["pitch_degrees"] = 2
        result = world_to_camera([10, 0, 2], [0, 0, 0], 1.347, camera)
        pitch = math.radians(2)
        self.assertAlmostEqual(result["depth_m"], math.cos(pitch) * 10 - math.sin(pitch) * 2)
        self.assertAlmostEqual(result["camera_xyz_forward_left_up_m"][2], math.sin(pitch) * 10 + math.cos(pitch) * 2)

    def test_width_axis_is_heading_plus_ninety_and_z_offset_not_added(self):
        result = project_housing(signal(), [0, 0, 0], 1.347, zero_camera())
        corners = [corner["world_xyz_m"] for corner in result["corners"]]
        self.assertAlmostEqual(max(c[1] for c in corners) - min(c[1] for c in corners), 1.)
        self.assertAlmostEqual(max(c[2] for c in corners), .25)
        self.assertEqual(result["signal_center_world_xyz_m"][2], 0)
        self.assertEqual(result["mgeo_z_offset_not_added"], 4.5)

    def test_behind_or_crossing_camera_plane_is_not_valid(self):
        for housing in (signal(center=(-10, 0, 0)), signal(center=(.1, 0, 0), heading=90, width=1)):
            result = project_housing(housing, [0, 0, 0], 1.347, zero_camera())
            self.assertFalse(result["valid"])
            self.assertFalse(result["all_corners_in_front"])
            self.assertIsNone(result["bbox_xyxy_integer_clipped"])

    def test_out_of_screen_is_invalid_and_partial_roi_is_clipped(self):
        result = project_housing(signal(center=(10, 9.9, 0), width=1), [0, 0, 0], 1.347, zero_camera())
        self.assertFalse(result["valid"])
        self.assertTrue(result["out_of_screen"])
        self.assertTrue(result["margin_clipped"])
        self.assertEqual(result["bbox_xyxy_integer_clipped"][0], 0)
        outside = project_housing(signal(center=(10, 30, 0)), [0, 0, 0], 1.347, zero_camera())
        self.assertFalse(outside["valid"])
        self.assertTrue(outside["completely_out_of_screen"])
        self.assertIsNone(outside["bbox_xyxy_integer_clipped"])

    def test_margin_expands_with_floor_and_ceil_without_mutating_inputs(self):
        housing, camera = signal(), zero_camera()
        original = copy.deepcopy(housing)
        result = project_housing(housing, [0, 0, 0], 1.347, camera, margin_px=4)
        raw, expanded = result["bbox_xyxy_float_no_margin"], result["bbox_xyxy_integer_unclipped"]
        self.assertEqual(expanded, [math.floor(raw[0] - 4), math.floor(raw[1] - 4),
                                    math.ceil(raw[2] + 4), math.ceil(raw[3] + 4)])
        self.assertEqual(housing, original)

    def test_invalid_dimensions_nan_margin_and_camera_rotation_are_rejected(self):
        for housing in (signal(width=0), signal(height=-1), signal(center=(float("nan"), 0, 0))):
            with self.assertRaises(ValueError):
                project_housing(housing, [0, 0, 0], 1.347)
        for margin in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                project_housing(signal(), [0, 0, 0], 1.347, margin_px=margin)
        camera = zero_camera()
        camera["yaw_degrees"] = 1
        with self.assertRaises(ValueError):
            project_housing(signal(), [0, 0, 0], 1.347, camera)

    def test_ego_snapshot_manifest_and_v4_pose_parsing(self):
        snapshot = {"ego": {"position_xyz_m": [1, 2, 3], "heading_degrees": 91.347,
                            "ego_header_stamp": {"secs": 12, "nsecs": 3}}}
        position, heading, metadata = parse_ego_document(snapshot)
        self.assertEqual(position, [1., 2., 3.])
        self.assertEqual(heading, 91.347)
        self.assertEqual(metadata["ego_header_stamp"], {"secs": 12, "nsecs": 3})
        self.assertEqual(parse_ego_document({"frames": [snapshot]}, 0)[:2], (position, heading))
        with self.assertRaises(ValueError):
            parse_ego_document({"frames": [snapshot]})
        self.assertEqual(parse_ego_document({"status": {"position": {"x": 1, "y": 2, "z": 3}, "heading": 91.347}})[:2],
                         (position, heading))

    def test_explicit_assets_are_read_only_and_output_is_exclusive(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mgeo = root / "traffic_light_set.json"
            mgeo.write_text(json.dumps([signal()]))
            original = mgeo.read_bytes()
            housing, source = load_signal(mgeo, "fixture")
            self.assertEqual(housing["idx"], "fixture")
            self.assertEqual(mgeo.read_bytes(), original)
            self.assertEqual(len(source["mgeo_sha256"]), 64)
            with self.assertRaises(ValueError):
                load_signal(mgeo, "wrong_map_id")
            output = root / "new.json"
            write_json({"candidate": True}, output)
            with self.assertRaises(FileExistsError):
                write_json({"overwrite": True}, output)
            self.assertEqual(json.loads(output.read_text()), {"candidate": True})

    def test_current_sensor_uses_fov_not_stored_focal_pixel(self):
        camera, provenance = load_fixed_camera()
        self.assertEqual(camera, fixed_camera())
        self.assertEqual(provenance["stored_focalLengthpixel_not_used"], 320.)


if __name__ == "__main__":
    unittest.main()
