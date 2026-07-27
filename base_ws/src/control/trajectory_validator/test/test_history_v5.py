#!/usr/bin/env python3

import csv
import tempfile
import unittest
from pathlib import Path

from trajectory_validator.csv_io import TRACKING_FIELDS, TrackingCsvWriter
from trajectory_validator.history_publisher import load_history_artifacts, sha256_file


class SchemaV5HistoryTest(unittest.TestCase):
    def test_compact_run_reconstructs_reference_and_ego_height(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectory_path = root / "trajectory.csv"
            with trajectory_path.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=[
                        "index",
                        "x",
                        "y",
                        "z",
                        "yaw",
                        "velocity_mps",
                        "acceleration_mps2",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "index": 0,
                        "x": 0.0,
                        "y": 0.0,
                        "z": 28.0,
                        "yaw": 0.0,
                        "velocity_mps": 5.0,
                        "acceleration_mps2": 0.0,
                    }
                )
                writer.writerow(
                    {
                        "index": 1,
                        "x": 10.0,
                        "y": 0.0,
                        "z": 28.5,
                        "yaw": 0.0,
                        "velocity_mps": 5.0,
                        "acceleration_mps2": 0.0,
                    }
                )

            run_directory = root / "run"
            run_directory.mkdir()
            tracking_path = run_directory / "tracking_samples.csv"
            sample = {
                field: (
                    0
                    if field == "sample_index"
                    else 9.0
                    if field == "matched_gt_station_m"
                    else False
                    if field in {
                        "control_pair_valid",
                        "pp_controller_valid",
                        "pp_debug_fresh",
                        "collision_detected",
                    }
                    else 0.0
                )
                for field in TRACKING_FIELDS
            }
            writer = TrackingCsvWriter(tracking_path)
            writer.write(sample)
            writer.close()

            metadata = {
                "schema_version": 5,
                "frame_id": "map",
                "reference_trajectory": {
                    "file": trajectory_path.name,
                    "sha256": sha256_file(trajectory_path),
                },
                "integrity": {
                    "tracking_samples_csv_sha256": sha256_file(tracking_path)
                },
            }
            trajectory, rows, responses = load_history_artifacts(
                root, run_directory, metadata
            )

            self.assertEqual(len(trajectory.points), 2)
            self.assertEqual(rows[0]["matched_gt_index"], "1")
            self.assertEqual(rows[0]["ego_z_m"], "28.5")
            self.assertEqual(responses, [])


if __name__ == "__main__":
    unittest.main()
