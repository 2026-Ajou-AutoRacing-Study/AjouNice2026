#!/usr/bin/env python3

import os
import sys
import unittest

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from motion_velocity_planner import MotionVelocityPlanner
from mvp_utils import path_cumulative_lengths, project_pose_to_path, set_velocity_window


def trajectory(velocities):
    message = Trajectory()
    message.header.frame_id = "map"
    for index, velocity in enumerate(velocities):
        point = TrajectoryPoint()
        point.pose.position.x = float(index)
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = float(velocity)
        message.points.append(point)
    return message


class TrajectoryInterfaceTest(unittest.TestCase):
    def test_lengths_use_native_trajectory_points(self):
        self.assertEqual(path_cumulative_lengths(trajectory([5.0] * 4)), [0.0, 1.0, 2.0, 3.0])

    def test_projection_uses_native_trajectory_points(self):
        message = trajectory([5.0] * 5)
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = 2.5
        pose.position.y = 0.4
        projection = project_pose_to_path(message, pose)
        self.assertAlmostEqual(projection.longitudinal_distance, 2.5)
        self.assertAlmostEqual(projection.signed_lateral_distance, 0.4)

    def test_velocity_window_supports_native_trajectory(self):
        output = set_velocity_window(trajectory([5.0] * 5), 1.0, 3.0, 2.0)
        self.assertEqual(
            [point.longitudinal_velocity_mps for point in output.points],
            [5.0, 2.0, 2.0, 2.0, 5.0],
        )

    def test_base_velocity_preserves_upstream_stop(self):
        planner = MotionVelocityPlanner.__new__(MotionVelocityPlanner)
        planner.use_velocity_limit = False
        planner.velocity_limit = None
        planner.default_velocity_mps = 5.0
        message = trajectory([8.0, 8.0, 0.0, 0.0])
        planner.apply_base_velocity(message)
        self.assertEqual(
            [point.longitudinal_velocity_mps for point in message.points],
            [5.0, 5.0, 0.0, 0.0],
        )


if __name__ == "__main__":
    unittest.main()
