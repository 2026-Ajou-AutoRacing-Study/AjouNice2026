#!/usr/bin/env python3
"""Initial-position/heading provenance contract without a running ROS master."""

import importlib.util
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "gnss_imu_initial_pose_publisher.py"
SPEC = importlib.util.spec_from_file_location("gnss_imu_initial_pose_publisher", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def status(stamp, x=0.0, y=0.0, z=0.0, heading=0.0):
    message = EgoVehicleStatus()
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.position.x = x
    message.position.y = y
    message.position.z = z
    message.heading = heading
    return message


def gnss_status(stamp, x=-0.13066733061446, y=117.5803152482179):
    message = Odometry()
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.header.frame_id = "map"
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    return message


def publisher(position_source="ego", heading_source="competition"):
    node = MODULE.GnssImuInitialPosePublisher.__new__(MODULE.GnssImuInitialPosePublisher)
    node.lock = threading.Lock()
    node.initialized = False
    node.source = position_source
    node.heading_source = heading_source
    node.latest_ego = None
    node.latest_heading = None
    node.latest_gnss = None
    node.max_pair_age_sec = 0.2
    node.max_official_gnss_age_sec = 0.5
    node.max_official_start_distance_m = 3.0
    node.max_official_yaw_error_deg = 1.0
    node.heading_in_degrees = True
    node.use_heading_correction = True
    node.heading_yaw_offset_rad = math.radians(-1.347)
    node.morai_map_frame = "morai_map"
    node.map_frame = "map"
    node.tf_timeout = 0.05
    node.official_position = np.array([-131.68979755061446, -428.3310229377821, 28.543960281954277])
    node.official_yaw_deg = 61.298126221
    transform = TransformStamped()
    transform.transform.translation.x = 131.559130220
    transform.transform.translation.y = 545.911338186
    transform.transform.rotation.w = 1.0
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = transform
    node.publish_once = Mock()
    return node


class InitialPoseProvenanceTest(unittest.TestCase):
    def test_ego_position_and_competition_heading_use_separate_sources(self):
        node = publisher()
        node.ego_callback(status(10.0, 12.0, 34.0, 28.0, 180.0))
        self.assertFalse(node.publish_once.called)
        node.heading_callback(status(10.04, heading=62.645126221))
        stamp, frame, pose = node.publish_once.call_args.args
        self.assertEqual(stamp, rospy.Time.from_sec(10.04))
        self.assertEqual(frame, "morai_map")
        np.testing.assert_array_equal(pose[0:3, 3], [12.0, 34.0, 28.0])
        self.assertAlmostEqual(math.degrees(tft.euler_from_matrix(pose)[2]), 61.298126221)

    def test_ego_position_waits_for_nearby_heading_stamp(self):
        node = publisher()
        node.ego_callback(status(10.0, 1.0, 2.0, 3.0))
        with patch.object(MODULE.rospy, "logwarn_throttle"):
            node.heading_callback(status(11.0, heading=62.645126221))
        self.assertFalse(node.publish_once.called)

    def test_official_position_uses_corrected_competition_heading_once(self):
        node = publisher(position_source="official_start")
        node.gnss_callback(gnss_status(10.0))
        with patch.object(MODULE.rospy, "loginfo_once"):
            node.heading_callback(status(10.0, heading=62.645126221))
        _, frame, pose = node.publish_once.call_args.args
        self.assertEqual(frame, "morai_map")
        np.testing.assert_allclose(pose[0:3, 3], node.official_position)
        self.assertAlmostEqual(math.degrees(tft.euler_from_matrix(pose)[2]), node.official_yaw_deg)

    def test_official_start_rejects_wrong_scenario_heading(self):
        node = publisher(position_source="official_start")
        node.gnss_callback(gnss_status(10.0))
        with patch.object(MODULE.rospy, "loginfo_once"), patch.object(MODULE.rospy, "logerr_throttle"):
            node.heading_callback(status(10.0, heading=100.0))
        self.assertFalse(node.publish_once.called)

    def test_official_start_rejects_missing_heading_correction(self):
        node = publisher(position_source="official_start")
        node.use_heading_correction = False
        node.gnss_callback(gnss_status(10.0))
        with patch.object(MODULE.rospy, "loginfo_once"), patch.object(MODULE.rospy, "logerr_throttle"):
            node.heading_callback(status(10.0, heading=62.645126221))
        self.assertFalse(node.publish_once.called)

    def test_official_start_waits_for_fresh_nearby_gnss(self):
        node = publisher(position_source="official_start")
        with patch.object(MODULE.rospy, "loginfo_once"), patch.object(MODULE.rospy, "logerr_throttle"):
            node.heading_callback(status(10.0, heading=62.645126221))
            self.assertFalse(node.publish_once.called)
            node.gnss_callback(gnss_status(10.0, x=10.0))
            self.assertFalse(node.publish_once.called)
            node.gnss_callback(gnss_status(10.1))
        self.assertTrue(node.publish_once.called)

    def test_official_start_is_transformed_to_map_once(self):
        node = publisher(position_source="official_start")
        node.publish_once = MODULE.GnssImuInitialPosePublisher.publish_once.__get__(node)
        node.map_frame = "map"
        node.tf_timeout = 0.05
        node.pose_covariance = [0.0] * 36
        node.publisher = Mock()
        node.gnss_callback(gnss_status(10.0))
        with patch.object(MODULE.rospy, "loginfo_once"):
            node.heading_callback(status(10.0, heading=62.645126221))
        message = node.publisher.publish.call_args.args[0]
        self.assertEqual(message.header.frame_id, "map")
        self.assertAlmostEqual(message.pose.pose.position.x, -0.13066733061446)
        self.assertAlmostEqual(message.pose.pose.position.y, 117.5803152482179)
        quaternion = message.pose.pose.orientation
        yaw = tft.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )[2]
        self.assertAlmostEqual(math.degrees(yaw), 61.298126221)

    def test_legacy_ego_heading_still_has_same_offset(self):
        node = publisher(heading_source="ego")
        node.ego_callback(status(10.0, 1.0, 2.0, 3.0, 63.862))
        pose = node.publish_once.call_args.args[2]
        self.assertAlmostEqual(math.degrees(tft.euler_from_matrix(pose)[2]), 62.515)


if __name__ == "__main__":
    unittest.main()
