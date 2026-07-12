#!/usr/bin/env python3

import os
import sys
import unittest

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import ObjectClassification, PredictedObject, PredictedObjects, Shape
from autoware_planning_msgs.msg import PathPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_path_tools import quaternion_from_yaw
from bpp_start_planner_tools import StartPlannerEngine, StartPlannerParameters


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in range(101):
        point = PathPoint()
        point.pose.position.x = float(x)
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = 5.0
        item = PathPointWithLaneId()
        item.point = point
        item.lane_ids = [10]
        path.points.append(item)
    path.left_bound = [Point(x=0.0, y=2.5), Point(x=100.0, y=2.5)]
    path.right_bound = [Point(x=0.0, y=-2.5), Point(x=100.0, y=-2.5)]
    return path


def make_odometry(x=5.0, y=-1.2, speed=0.0):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation = quaternion_from_yaw(0.0)
    message.twist.twist.linear.x = speed
    return message


def make_object(x, y, speed=0.0):
    obj = PredictedObject()
    obj.object_id.uuid = [1] + [0] * 15
    classification = ObjectClassification()
    classification.label = ObjectClassification.CAR
    classification.probability = 1.0
    obj.classification.append(classification)
    obj.shape.type = Shape.BOUNDING_BOX
    obj.shape.dimensions.x = 4.5
    obj.shape.dimensions.y = 2.0
    obj.kinematics.initial_pose_with_covariance.pose.position.x = x
    obj.kinematics.initial_pose_with_covariance.pose.position.y = y
    obj.kinematics.initial_pose_with_covariance.pose.orientation = quaternion_from_yaw(0.0)
    obj.kinematics.initial_twist_with_covariance.twist.linear.x = speed
    message = PredictedObjects()
    message.objects = [obj]
    return message


class StartPlannerEngineTest(unittest.TestCase):
    def setUp(self):
        self.path = make_path()
        self.engine = StartPlannerEngine(StartPlannerParameters())

    def test_shift_pull_out_merges_right_side_start_to_centerline(self):
        result = self.engine.plan(self.path, PredictedObjects(), make_odometry())
        self.assertTrue(result.requested)
        self.assertTrue(result.safe)
        self.assertEqual(result.direction, "left")
        self.assertAlmostEqual(result.candidate_path.points[5].point.pose.position.y, -1.2, delta=0.05)
        self.assertAlmostEqual(result.candidate_path.points[-1].point.pose.position.y, 0.0, delta=0.01)
        self.assertGreater(result.finish_distance, result.start_distance)

    def test_does_not_execute_on_centerline(self):
        result = self.engine.plan(
            self.path, PredictedObjects(), make_odometry(y=0.1)
        )
        self.assertFalse(result.requested)

    def test_static_object_on_pull_out_path_blocks_departure(self):
        result = self.engine.plan(
            self.path, make_object(10.0, -1.0), make_odometry()
        )
        self.assertTrue(result.requested)
        self.assertFalse(result.safe)
        self.assertEqual(result.candidates[0].failed_reason, "static_object_clearance")

    def test_fast_rear_object_blocks_departure(self):
        result = self.engine.plan(
            self.path, make_object(1.0, -1.2, speed=5.0), make_odometry()
        )
        self.assertFalse(result.safe)
        self.assertEqual(result.candidates[0].failed_reason, "rear_dynamic_object_risk")


if __name__ == "__main__":
    unittest.main()
