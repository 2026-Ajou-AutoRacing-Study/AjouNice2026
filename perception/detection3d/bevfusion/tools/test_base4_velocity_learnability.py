#!/usr/bin/env python3

import importlib.util
import pickle
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name(
    "diagnose_base4_velocity_learnability.py")
SPEC = importlib.util.spec_from_file_location("velocity_diagnostic", MODULE_PATH)
DIAGNOSTIC = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIAGNOSTIC)


def row(sample, scene, timestamp, x, speed):
    identity = np.eye(4).tolist()
    return {
        "sample_idx": sample,
        "scene_id": scene,
        "timestamp": timestamp,
        "lidar2global": identity,
        "instances": [{
            "instance_id": 1,
            "asset_name": "car",
            "num_lidar_pts": 20,
            "bbox_3d": [x, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0, speed, 0.0],
        }],
    }


class VelocityLearnabilityDiagnosticTest(unittest.TestCase):
    def test_constant_motion_has_zero_oracle_error(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            train = [
                row("a", "scene-a", 0.0, 0.0, 10.0),
                row("b", "scene-a", 0.1, 1.0, 10.0),
                row("c", "scene-a", 0.2, 2.0, 10.0),
            ]
            for split, rows in (("train", train), ("val", []), ("test", [])):
                with (root / f"morai_base3_infos_{split}.pkl").open("wb") as stream:
                    pickle.dump({"data_list": rows}, stream)
            report = DIAGNOSTIC.diagnose(root, horizons=(1, 2))
            self.assertEqual(report["splits"]["train"]["scenes"], 1)
            self.assertEqual(
                report["per_scene_oracle"]["train"]
                ["scenes_with_span_over_0_5_mps"],
                0,
            )
            self.assertAlmostEqual(
                report["gt_centre_displacement_oracle"]["train"]["1"]
                ["vector_error_mps"]["maximum"],
                0.0,
            )
            self.assertAlmostEqual(
                report["gt_centre_displacement_oracle"]["train"]["2"]
                ["speed_mae_mps"]["mean"],
                0.0,
            )


if __name__ == "__main__":
    unittest.main()
