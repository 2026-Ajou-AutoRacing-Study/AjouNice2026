"""Tests for the server-side Base3 point payload contract."""

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

    def test_rejects_old_five_feature_payload(self):
        with self.assertRaisesRegex(ValueError, "four float32"):
            parse_points(
                {"points_blob": "points", "point_dim": 5},
                {"points": np.zeros((2, 5), dtype="<f4").tobytes()},
            )


if __name__ == "__main__":
    unittest.main()
