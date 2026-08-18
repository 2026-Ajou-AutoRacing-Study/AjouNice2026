import unittest

import numpy as np

from perception_object_pipeline.rolling_lidar import (
    representative_detection_time_offset,
)


class RollingLidarTimingTest(unittest.TestCase):
    def test_uses_median_of_per_box_medians(self):
        points = np.asarray(
            [
                [0.0, 0.0, 0.0, 1.0],
                [0.1, 0.0, 0.0, 1.0],
                [-0.1, 0.0, 0.0, 1.0],
                [10.0, 0.0, 0.0, 1.0],
                [10.1, 0.0, 0.0, 1.0],
                [9.9, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        offsets = np.asarray(
            [-0.10, -0.09, -0.08, -0.02, -0.01, 0.0], dtype=np.float32
        )
        boxes = np.asarray(
            [
                [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0],
                [10.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0],
            ],
            dtype=np.float32,
        )
        result = representative_detection_time_offset(points, offsets, boxes)
        self.assertAlmostEqual(result, -0.05, places=6)

    def test_falls_back_to_scan_median(self):
        points = np.zeros((3, 4), dtype=np.float32)
        offsets = np.asarray([-0.12, -0.08, 0.0], dtype=np.float32)
        boxes = np.asarray(
            [[20.0, 20.0, 0.0, 1.0, 1.0, 1.0, 0.0]], dtype=np.float32
        )
        result = representative_detection_time_offset(points, offsets, boxes)
        self.assertAlmostEqual(result, -0.08, places=6)

    def test_missing_time_is_header_offset_zero(self):
        points = np.zeros((2, 4), dtype=np.float32)
        offsets = np.asarray([np.nan, np.nan], dtype=np.float32)
        result = representative_detection_time_offset(
            points, offsets, np.empty((0, 7), dtype=np.float32)
        )
        self.assertEqual(result, 0.0)


if __name__ == "__main__":
    unittest.main()
