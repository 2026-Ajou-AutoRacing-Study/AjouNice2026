import unittest

from tracking_prediction_evaluation.trajectory import (
    TrajectoryPoint,
    interpolate_point,
    reconstruct_trajectory_segments,
)


def point(timestamp, x, yaw=0.0):
    return TrajectoryPoint(
        timestamp=timestamp,
        x=x,
        y=0.0,
        z=0.0,
        yaw=yaw,
        longitudinal_velocity=1.0,
        lateral_velocity=0.0,
        longitudinal_acceleration=0.0,
        lateral_acceleration=0.0,
        class_name="vehicle",
        object_id=1,
        asset_name="synthetic",
    )


class TrajectoryTest(unittest.TestCase):
    def test_interpolation_uses_timestamp_bracket(self):
        result = interpolate_point([point(0.0, 0.0), point(0.2, 2.0)], 0.1, 0.3)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.x, 1.0)

    def test_interpolation_rejects_large_gap(self):
        result = interpolate_point([point(0.0, 0.0), point(1.0, 1.0)], 0.5, 0.3)
        self.assertIsNone(result)

    def test_reconstruction_ignores_swapped_source_ids(self):
        frames = []
        for frame_index in range(12):
            timestamp = frame_index * 0.1
            left = point(timestamp, timestamp)
            right = point(timestamp, 10.0 + timestamp)
            # MORAI source IDs swap, but spatial identities remain continuous.
            left = TrajectoryPoint(
                **{**left.__dict__, "object_id": 1 if frame_index % 2 == 0 else 9,
                   "asset_name": "left"}
            )
            right = TrajectoryPoint(
                **{**right.__dict__, "object_id": 9 if frame_index % 2 == 0 else 1,
                   "asset_name": "right"}
            )
            frames.append((timestamp, [left, right]))

        segments = reconstruct_trajectory_segments(frames, minimum_points=8)

        self.assertEqual(len(segments), 2)
        self.assertEqual([len(segment.points) for segment in segments], [12, 12])
        starts = sorted(segment.points[0].x for segment in segments)
        ends = sorted(segment.points[-1].x for segment in segments)
        self.assertEqual(starts, [0.0, 10.0])
        self.assertAlmostEqual(ends[0], 1.1)
        self.assertAlmostEqual(ends[1], 11.1)


if __name__ == "__main__":
    unittest.main()
