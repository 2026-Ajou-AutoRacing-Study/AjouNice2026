#!/usr/bin/env python3

import csv
import tempfile
import unittest
from pathlib import Path

from trajectory_validator.csv_io import (
    TRACKING_FIELDS,
    TrackingCsvWriter,
    load_tracking_csv,
)


class TrackingCsvTest(unittest.TestCase):
    def sample(self, index):
        return {
            field: (
                index
                if field == "sample_index"
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

    def test_writer_uses_exact_compact_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tracking_samples.csv"
            writer = TrackingCsvWriter(path)
            writer.write(self.sample(0))
            writer.close()

            with path.open(newline="") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(reader.fieldnames, TRACKING_FIELDS)
                self.assertEqual(len(list(reader)), 1)
            self.assertEqual(load_tracking_csv(path)[0]["sample_index"], "0")

    def test_reader_rejects_non_contiguous_indices(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tracking_samples.csv"
            writer = TrackingCsvWriter(path)
            writer.write(self.sample(1))
            writer.close()
            with self.assertRaisesRegex(ValueError, "not contiguous"):
                load_tracking_csv(path)


if __name__ == "__main__":
    unittest.main()
