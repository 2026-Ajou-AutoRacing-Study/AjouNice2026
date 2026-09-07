#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools/temporal_geometry.py"
SPEC = importlib.util.spec_from_file_location("temporal_geometry", MODULE_PATH)
geometry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(geometry)


def frame(timestamp, x=0.0, y=0.0, yaw_deg=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
    return {
        "triggered_unix": timestamp,
        "ego": {
            "timestamp": timestamp,
            "position_m": {"x": x, "y": y, "z": 0.0},
            "heading_deg": yaw_deg,
            "derived_velocity_mps": {"x": vx, "y": vy, "z": 0.0},
            "derived_yaw_rate_dps": yaw_rate,
            "velocity_raw": {"x": vx, "y": vy, "z": 0.0},
            "speed_raw": float(np.hypot(vx, vy)),
        },
    }


class TemporalGeometryTest(unittest.TestCase):
    def test_clockwise_azimuth_offsets(self):
        angles = np.deg2rad([0.0, 270.0, 180.0, 90.0])
        points = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(4)))
        offsets = geometry.estimate_clockwise_point_offsets(points, 0.08)
        np.testing.assert_allclose(offsets, [0.0, 0.02, 0.04, 0.06], atol=1.0e-8)

    def test_sweep_alignment_uses_ego_poses(self):
        # Static world point x=10: source Ego x=1 observes x=9; target Ego x=2
        # must observe x=8.
        points = np.asarray([[9.0, 0.0, 0.0, 1.0]], dtype=np.float32)
        output = geometry.transform_points_to_target(
            points,
            frame(0.0, x=1.0),
            frame(1.0, x=2.0),
            np.zeros(1),
            lidar_position=(0.0, 0.0, 0.0),
        )
        np.testing.assert_allclose(output[0, :3], [8.0, 0.0, 0.0], atol=1.0e-6)

    def test_ego_deskew_restores_static_world_point(self):
        offsets = np.asarray([0.0, 0.05])
        # Ego moves +x at 2m/s. A static x=10 point is seen as 10 and 9.9.
        points = np.asarray([[10.0, 0.0, 0.0, 1.0], [9.9, 0.0, 0.0, 1.0]])
        source = frame(0.0, vx=2.0)
        raw = geometry.transform_points_to_target(
            points, source, source, offsets, lidar_position=(0.0, 0.0, 0.0)
        )
        corrected = geometry.transform_points_to_target(
            points,
            source,
            source,
            offsets,
            lidar_position=(0.0, 0.0, 0.0),
            deskew_ego_motion=True,
        )
        np.testing.assert_allclose(raw[:, 0], [10.0, 9.9], atol=1.0e-6)
        np.testing.assert_allclose(corrected[:, 0], [10.0, 10.0], atol=1.0e-6)

    def test_temporal_feature_is_actual_point_lag(self):
        points = np.asarray([[1.0, 0.0, 0.0, 1.0]], dtype=np.float32)
        output = geometry.temporalize_sweep(
            points,
            frame(9.0),
            frame(10.0),
            0.08,
            lidar_position=(0.0, 0.0, 0.0),
        )
        self.assertEqual(output.shape, (1, 5))
        self.assertAlmostEqual(float(output[0, -1]), -1.0, places=6)

    def test_simulation_time_uses_motion_instead_of_wall_time(self):
        left = frame(100.0, x=0.0, vx=10.0)
        right = frame(100.5, x=0.6, vx=10.0)
        left.update({"scene_id": "s", "frame_id": "f0", "frame_index": 0})
        right.update({"scene_id": "s", "frame_id": "f1", "frame_index": 1})
        diagnostics = geometry.annotate_simulation_times([left, right])
        self.assertAlmostEqual(right["simulation_time_sec"], 0.06, places=6)
        self.assertAlmostEqual(diagnostics[0]["wall_to_sim_ratio"], 0.12, places=6)


if __name__ == "__main__":
    unittest.main()
