import unittest

import numpy as np

from perception_object_pipeline.rolling_lidar import (
    PlanarPoseBuffer,
    deskew_points_to_reference_pose,
    lidar_world_pose_at_time,
    representative_detection_time_offset,
    scan_time_contract,
    transform_boxes_between_lidar_times,
    transform_points_between_lidar_times,
)


class RollingLidarTimingTest(unittest.TestCase):
    def test_scan_time_contract_uses_scan_end(self):
        contract = scan_time_contract(
            np.asarray([0.001, 0.050, 0.099], dtype=np.float32),
            max_span_sec=0.15,
            reference="end",
        )
        self.assertIsNotNone(contract)
        self.assertAlmostEqual(contract["span_sec"], 0.098, places=6)
        self.assertAlmostEqual(contract["reference_offset_sec"], 0.099, places=6)

    def test_scan_time_contract_rejects_invalid_scans(self):
        self.assertIsNone(scan_time_contract([0.0, 0.0]))
        self.assertIsNone(scan_time_contract([0.0, float("nan")]))
        self.assertIsNone(scan_time_contract([0.0, 0.16], max_span_sec=0.15))

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

    def test_past_sweep_is_reexpressed_in_current_lidar_frame(self):
        poses = self.poses(
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0, 0.0),
        )
        points = np.asarray(
            [[5.0, 0.0, 0.0, 0.7, 0.1, 0.9]], dtype=np.float32
        )
        transformed = transform_points_between_lidar_times(
            points,
            source_time_sec=0.0,
            target_time_sec=1.0,
            pose_samples=poses,
            maximum_extrapolation_sec=0.0,
        )
        self.assertIsNotNone(transformed)
        np.testing.assert_allclose(transformed[:, :3], [[4.0, 0.0, 0.0]])
        np.testing.assert_allclose(transformed[:, 3:], points[:, 3:])

    def test_lidar_world_pose_uses_interpolated_ego_and_extrinsic(self):
        poses = self.poses(
            (0.0, 0.0, 0.0, 1.0, 0.0),
            (2.0, 2.0, 0.0, 1.0, np.pi / 2.0),
        )
        pose = lidar_world_pose_at_time(
            poses,
            1.0,
            lidar_position=(1.0, 0.0, 0.5),
            lidar_yaw=0.1,
            maximum_extrapolation_sec=0.0,
        )
        self.assertIsNotNone(pose)
        expected_yaw = np.pi / 4.0 + 0.1
        self.assertAlmostEqual(float(pose[0]), 1.0 + np.cos(np.pi / 4.0), places=6)
        self.assertAlmostEqual(float(pose[1]), np.sin(np.pi / 4.0), places=6)
        self.assertAlmostEqual(float(pose[2]), 1.5, places=6)
        self.assertAlmostEqual(float(pose[3]), expected_yaw, places=6)

    def test_velocity_vector_rotates_with_lidar_axes(self):
        poses = self.poses(
            (1.0, 0.0, 0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0, 0.0, np.pi / 2.0),
        )
        boxes = np.asarray(
            [[5.0, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0, 2.0, 0.0]],
            dtype=np.float32,
        )
        transformed = transform_boxes_between_lidar_times(
            boxes, 1.0, 2.0, poses
        )
        self.assertAlmostEqual(float(transformed[0, 7]), 0.0, places=6)
        self.assertAlmostEqual(float(transformed[0, 8]), -2.0, places=6)

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
