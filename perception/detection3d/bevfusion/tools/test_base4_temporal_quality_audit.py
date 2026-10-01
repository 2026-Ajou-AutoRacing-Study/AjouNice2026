#!/usr/bin/env python3

import importlib.util
import pickle
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("audit_base4_temporal_quality.py")
SPEC = importlib.util.spec_from_file_location("base4_quality_audit", MODULE_PATH)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def row(sample, scene, timestamp, x, speed):
    identity = np.eye(4).tolist()
    return {
        "sample_idx": sample,
        "scene_id": scene,
        "timestamp": timestamp,
        "lidar2global": identity,
        "lidar_sweeps": [],
        "instances": [{
            "instance_id": 7,
            "bbox_label_3d": 0,
            "bbox_3d": [x, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0, speed, 0.0],
            "velocity_source": "test",
        }],
    }


class Base4TemporalQualityAuditTest(unittest.TestCase):
    def write_infos(self, root, train, val=None, test=None):
        metainfo = {
            "classes": AUDIT.CLASSES,
            "box_dim": 9,
            "causal_sweep_count": 1,
        }
        for split, rows in (
            ("train", train), ("val", val or []), ("test", test or [])
        ):
            with (root / f"morai_base3_infos_{split}.pkl").open("wb") as f:
                pickle.dump({"metainfo": metainfo, "data_list": rows}, f)

    def run_audit(self, root):
        return AUDIT.audit(
            root,
            velocity_warn_mps=2.0,
            velocity_reject_mps=5.0,
            maximum_pair_gap_sec=0.35,
            maximum_expected_vehicle_speed_mps=35.0,
            maximum_state_age_sec=0.10,
            maximum_camera_age_sec=0.10,
        )

    def test_consistent_mps_velocity_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_infos(root, [
                row("s0", "scene-a", 0.0, 0.0, 10.0),
                row("s1", "scene-a", 0.1, 1.0, 10.0),
            ])
            report = self.run_audit(root)
            self.assertTrue(report["valid"])
            self.assertAlmostEqual(
                report["bbox_center_velocity_consistency_mps"]["maximum"],
                0.0,
            )

    def test_kph_as_mps_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_infos(root, [
                row("s0", "scene-a", 0.0, 0.0, 40.0),
                row("s1", "scene-a", 0.1, 1.111111, 40.0),
            ])
            report = self.run_audit(root)
            self.assertFalse(report["valid"])
            self.assertEqual(report["excessive_vehicle_speed"]["count"], 2)
            self.assertGreater(
                report["bbox_center_velocity_consistency_mps"]["p95"],
                20.0,
            )

    def test_scene_split_leakage_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_infos(
                root,
                [row("train", "shared", 0.0, 0.0, 10.0)],
                [row("val", "shared", 1.0, 10.0, 10.0)],
            )
            report = self.run_audit(root)
            self.assertFalse(report["valid"])
            self.assertEqual(report["split_leakage"]["shared"], ["train", "val"])


if __name__ == "__main__":
    unittest.main()
