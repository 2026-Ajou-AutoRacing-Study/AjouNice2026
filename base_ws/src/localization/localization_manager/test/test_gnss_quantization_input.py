#!/usr/bin/env python3
"""Small contract tests for the experimental GNSS cell index."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "morai_gps_to_local_odometry.py"
SPEC = importlib.util.spec_from_file_location("morai_gps_to_local_odometry", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GnssQuantizationInputTest(unittest.TestCase):
    def test_repeated_quantized_coordinates_keep_the_same_cell(self):
        step = 1.0 / 600000.0
        self.assertEqual(
            MODULE.quantized_cell(37.5, 127.0, step),
            MODULE.quantized_cell(37.5, 127.0, step),
        )

    def test_one_lattice_step_changes_only_its_axis(self):
        step = 1.0 / 600000.0
        first = MODULE.quantized_cell(37.5, 127.0, step)
        second = MODULE.quantized_cell(37.5 + step, 127.0, step)
        self.assertEqual(second, (first[0] + 1, first[1]))

    def test_default_mode_never_suppresses_a_valid_fix(self):
        self.assertTrue(MODULE.should_publish_quantized_fix("off", (1, 2), (1, 2)))

    def test_cell_transition_suppresses_only_repeated_cell(self):
        self.assertTrue(MODULE.should_publish_quantized_fix("cell_transition", (1, 2), None))
        self.assertFalse(MODULE.should_publish_quantized_fix("cell_transition", (1, 2), (1, 2)))
        self.assertTrue(MODULE.should_publish_quantized_fix("cell_transition", (1, 3), (1, 2)))


if __name__ == "__main__":
    unittest.main()
