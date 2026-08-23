#!/usr/bin/env python3

import importlib.util
import math
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "temporal_training_index",
    ROOT / "dataset/morai_od/scripts/build_temporal_training_index.py",
)
D3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D3)


def pose(x, y, heading=0.0):
    yaw = math.radians(heading)
    c = math.cos(yaw)
    s = math.sin(yaw)
    ego_to_global = [
        [c, -s, 0.0, x],
        [s, c, 0.0, y],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    global_to_ego = [
        [c, s, 0.0, -(c * x + s * y)],
        [-s, c, 0.0, s * x - c * y],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return {
        "ego": {"heading_deg": heading},
        "transforms": {
            "ego_to_global": ego_to_global,
            "global_to_ego": global_to_ego,
        },
    }


def actor(unique_id, y, raw_kmh=36.0):
    return {
        "class": "vehicle",
        "unique_id": unique_id,
        "name": "test_vehicle",
        "heading_deg": 90.0,
        "position_m": {"x": 0.0, "y": y, "z": 0.0},
        "relative_to_ego": {"position_m": {"x": 0.0, "y": y, "z": 0.0}},
        "morai_velocity_raw": {"x": raw_kmh, "y": 0.0, "z": 0.0},
    }


class TemporalTrainingIndexTest(unittest.TestCase):
    def test_relative_transform_moves_old_point_into_target_frame(self):
        sweep = pose(0.0, 0.0)
        target = pose(2.0, 0.0)
        identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        calibration = {"lidar": {"lidar_to_ego": identity, "ego_to_lidar": identity}}
        ego_transform, lidar_transform = D3.relative_transform(
            sweep, target, calibration
        )
        for transform in (ego_transform, lidar_transform):
            point = D3.transform_point(transform, {"x": 10.0, "y": 1.0, "z": 0.0})
            self.assertAlmostEqual(point["x"], 8.0)
            self.assertAlmostEqual(point["y"], 1.0)

    def test_object_local_velocity_is_rotated_into_world(self):
        frames = []
        for index, y in enumerate((0.0, 1.0, 2.0)):
            frame = pose(0.0, 0.0)
            frame.update(
                {
                    "sequence_time_sec": index * 0.1,
                    "mission_family": "roundabout_circulation",
                    "objects": {"actors": [actor(7, y)]},
                }
            )
            frames.append(frame)
        args = SimpleNamespace(
            minimum_track_frames=3,
            maximum_state_age_sec=0.1,
            maximum_derived_speed_mps=45.0,
            speed_error_base_mps=0.75,
            speed_error_ratio=0.30,
            maximum_direction_error_deg=45.0,
            maximum_raw_speed_mps=60.0,
        )
        label = D3.make_velocity_label(
            frames[2],
            frames,
            2,
            7,
            "Vehicle",
            {"actors": [actor(7, 2.0)]},
            0.005,
            Counter({("vehicle", 7): 3}),
            args,
        )
        self.assertEqual(label["velocity_source"], "morai_raw_local_rotated")
        self.assertTrue(label["velocity_valid"])
        self.assertAlmostEqual(label["velocity_target_world_mps"]["x"], 0.0, places=6)
        self.assertAlmostEqual(label["velocity_target_world_mps"]["y"], 10.0, places=6)
        self.assertAlmostEqual(label["derived_velocity_world_mps"]["y"], 10.0, places=6)

    def test_lidar_instance_parser_reads_sensor_frame_centre_and_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "Instance.txt"
            path.write_text(
                "Vehicle 1.0 2.0 3.0 0 0 0 4 2 2 0 0 0 0 17\n",
                encoding="utf-8",
            )
            centres = D3.parse_lidar_centres(path)
        self.assertEqual(centres[17]["class"], "Vehicle")
        self.assertEqual(centres[17]["centre"], {"x": 1.0, "y": 2.0, "z": 3.0})


if __name__ == "__main__":
    unittest.main()
