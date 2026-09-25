#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest

from nav_msgs.msg import Odometry

SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "gnss_imu_kinematic_state_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("gnss_imu_publisher", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GnssImuContractTest(unittest.TestCase):
    def test_missing_height_blocks_initial_output(self):
        self.assertEqual(MODULE.z_sample_state(False, 0.0, 0.5), "missing")

    def test_fresh_height_is_used(self):
        self.assertEqual(MODULE.z_sample_state(True, 0.49, 0.5), "fresh")

    def test_stale_height_is_held_instead_of_blocking_planar_state(self):
        self.assertEqual(MODULE.z_sample_state(True, 10.93, 0.5), "held")

    def test_future_height_is_diagnostic_hold_not_silent_freshness(self):
        self.assertEqual(MODULE.z_sample_state(True, -1.0, 0.5), "held")

    def test_non_finite_height_age_is_invalid(self):
        self.assertEqual(
            MODULE.z_sample_state(True, float("nan"), 0.5), "invalid"
        )

    def test_local_pose_is_extrapolated_to_global_timestamp(self):
        message = Odometry()
        message.header.stamp = MODULE.rospy.Time.from_sec(10.0)
        message.pose.pose.position.x = 2.0
        message.pose.pose.position.y = 3.0
        message.pose.pose.orientation.z = math.sin(math.pi / 4.0)
        message.pose.pose.orientation.w = math.cos(math.pi / 4.0)
        message.twist.twist.linear.x = 10.0
        message.twist.twist.angular.z = 0.2

        x, y, z, yaw = MODULE.extrapolated_planar_pose(message, 10.02)

        self.assertAlmostEqual(x, 2.0, places=9)
        self.assertAlmostEqual(y, 3.2, places=9)
        self.assertAlmostEqual(z, 0.0, places=9)
        self.assertAlmostEqual(yaw, math.pi / 2.0 + 0.004, places=9)

    def test_pose_composition_round_trip(self):
        map_to_base = Odometry().pose.pose
        map_to_base.position.x = 101.0
        map_to_base.position.y = 52.0
        map_to_base.orientation.z = math.sin(0.15)
        map_to_base.orientation.w = math.cos(0.15)
        local_pose = (2.0, 3.0, 0.0, 0.1)

        alignment = MODULE.compose_planar_pose_map_to_odom(
            map_to_base, local_pose
        )
        cosine = math.cos(alignment[3])
        sine = math.sin(alignment[3])
        x = alignment[0] + cosine * local_pose[0] - sine * local_pose[1]
        y = alignment[1] + sine * local_pose[0] + cosine * local_pose[1]

        self.assertAlmostEqual(x, map_to_base.position.x, places=9)
        self.assertAlmostEqual(y, map_to_base.position.y, places=9)
        self.assertAlmostEqual(
            MODULE.normalized_angle(alignment[3] + local_pose[3]),
            0.3,
            places=9,
        )

if __name__ == "__main__":
    unittest.main()
