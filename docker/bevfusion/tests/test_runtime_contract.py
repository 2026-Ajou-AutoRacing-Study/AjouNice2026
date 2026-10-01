"""Tests for the server-side single-frame and temporal point contracts."""

import unittest

import numpy as np

from app.inference_server import parse_points


class RuntimeContractTest(unittest.TestCase):
    def test_parses_xyzi_payload(self):
        expected = np.arange(12, dtype="<f4").reshape(3, 4)
        actual = parse_points(
            {"points_blob": "points", "point_dim": 4},
            {"points": expected.tobytes()},
        )
        np.testing.assert_array_equal(actual, expected)

    def test_parses_xyzi_time_payload(self):
        expected = np.arange(15, dtype="<f4").reshape(3, 5)
        actual = parse_points(
            {"points_blob": "points", "point_dim": 5},
            {"points": expected.tobytes()},
        )
        np.testing.assert_array_equal(actual, expected)

    def test_rejects_unsupported_feature_width(self):
        with self.assertRaisesRegex(ValueError, "4 or 5"):
            parse_points(
                {"points_blob": "points", "point_dim": 6},
                {"points": np.zeros((2, 6), dtype="<f4").tobytes()},
            )


if __name__ == "__main__":
    unittest.main()
