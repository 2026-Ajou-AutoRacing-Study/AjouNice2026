import unittest

import numpy as np

from perception_object_pipeline.rolling_lidar import (
    PlanarPoseBuffer,
    deskew_points_to_reference_pose,
    representative_detection_time_offset,
    transform_boxes_between_lidar_times,
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


class RollingLidarDeskewTest(unittest.TestCase):
    @staticmethod
    def poses(*rows):
        buffer = PlanarPoseBuffer(duration_sec=10.0)
        for row in rows:
            buffer.add(*row)
        return buffer.snapshot()

    def test_translation_aligns_static_world_point(self):
        poses = self.poses(
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0, 0.0),
            (2.0, 2.0, 0.0, 0.0, 0.0),
        )
        # The same world point x=10 is observed at two acquisition times.
        points = np.asarray(
            [[9.0, 0.0, 0.0, 1.0], [8.0, 0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        result = deskew_points_to_reference_pose(
            points,
            np.asarray([-1.0, 0.0], dtype=np.float32),
            scan_stamp_sec=2.0,
            reference_time_offset_sec=0.0,
            pose_samples=poses,
        )
        self.assertIsNotNone(result)
        corrected, displacement, _ = result
        np.testing.assert_allclose(corrected[:, 0], [8.0, 8.0], atol=1e-6)
        np.testing.assert_allclose(displacement, [1.0, 0.0], atol=1e-6)

    def test_rotation_aligns_static_world_point(self):
        poses = self.poses(
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0, 0.0, np.pi / 2.0),
        )
        points = np.asarray(
            [[10.0, 0.0, 0.0, 1.0], [0.0, -10.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        result = deskew_points_to_reference_pose(
            points,
            np.asarray([-1.0, 0.0], dtype=np.float32),
            scan_stamp_sec=2.0,
            reference_time_offset_sec=0.0,
            pose_samples=poses,
        )
        corrected, _, _ = result
        np.testing.assert_allclose(corrected[:, :2], [[0.0, -10.0]] * 2, atol=1e-6)

    def test_box_is_reexpressed_at_target_lidar_time(self):
        poses = self.poses(
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0, 0.0),
            (2.0, 2.0, 0.0, 0.0, 0.0),
        )
        boxes = np.asarray(
            [[9.0, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0]], dtype=np.float32
        )
        transformed = transform_boxes_between_lidar_times(
            boxes, 1.0, 2.0, poses
        )
        self.assertIsNotNone(transformed)
        self.assertAlmostEqual(float(transformed[0, 0]), 8.0, places=6)
        self.assertAlmostEqual(float(transformed[0, 6]), 0.0, places=6)

    def test_rejects_pose_gap_outside_tolerance(self):
        poses = self.poses(
            (1.0, 0.0, 0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = deskew_points_to_reference_pose(
            np.zeros((1, 4), dtype=np.float32),
            np.asarray([-2.0], dtype=np.float32),
            scan_stamp_sec=2.0,
            reference_time_offset_sec=0.0,
            pose_samples=poses,
            maximum_extrapolation_sec=0.01,
        )
        self.assertIsNone(result)

    def test_bounded_causal_pose_extrapolation(self):
        poses = self.poses(
            (1.00, 0.0, 0.0, 0.0, 0.0),
            (1.02, 0.2, 0.0, 0.0, 0.0),
        )
        # At 10 m/s, the pose at 1.04 s is x=0.4 m. A static point at
        # world x=10 therefore appears at x=9.6 m in that LiDAR frame.
        result = deskew_points_to_reference_pose(
            np.asarray([[9.8, 0.0, 0.0, 1.0]], dtype=np.float32),
            np.asarray([0.0], dtype=np.float32),
            scan_stamp_sec=1.02,
            reference_time_offset_sec=0.02,
            pose_samples=poses,
            maximum_extrapolation_sec=0.03,
        )
        self.assertIsNotNone(result)
        corrected, _, _ = result
        self.assertAlmostEqual(float(corrected[0, 0]), 9.6, places=5)


if __name__ == "__main__":
    unittest.main()
