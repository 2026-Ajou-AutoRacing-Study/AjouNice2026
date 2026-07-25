#!/usr/bin/env python3

import importlib.util
import os
import threading
import unittest
from unittest.mock import Mock

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


def load_node_module():
    script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__), "..", "scripts", "mission_planner_rviz_input.py"
        )
    )
    spec = importlib.util.spec_from_file_location("mission_planner_rviz_input", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MissionPlannerRvizInputTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_module = load_node_module()

    def make_node(self):
        node = self.node_module.MissionPlannerRvizInput.__new__(
            self.node_module.MissionPlannerRvizInput
        )
        node.frame_id = "map"
        node.default_z = 28.5
        node.lock = threading.RLock()
        node.start_pose = None
        node.current_pose = None
        node.goal_pose = None
        return node

    def test_normalized_pose_sets_map_height_and_valid_orientation(self):
        node = self.make_node()

        normalized = node.normalized_pose(Pose())

        self.assertEqual(normalized.position.z, 28.5)
        self.assertEqual(normalized.orientation.w, 1.0)

    def test_fixed_start_and_goal_markers_are_published(self):
        node = self.make_node()
        node.start_pose = Pose()
        node.goal_pose = Pose()
        node.marker_pub = Mock()
        node.arrow_marker = lambda marker_id, namespace, _pose, _color: Marker(
            ns=namespace, id=marker_id
        )

        node.publish_markers()

        marker_array = node.marker_pub.publish.call_args.args[0]
        self.assertEqual(
            [(marker.ns, marker.id) for marker in marker_array.markers],
            [("rviz_start", 0), ("rviz_goal", 1)],
        )

    def test_fixed_ego_odometry_has_zero_velocity(self):
        node = self.make_node()
        node.current_pose = Pose()
        node.current_pose.position.x = 12.0
        node.odometry_pub = Mock()

        node.publish_odometry(rospy.Time(1.0))

        message = node.odometry_pub.publish.call_args.args[0]
        self.assertEqual(message.pose.pose.position.x, 12.0)
        self.assertEqual(message.twist.twist.linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
