#!/usr/bin/env python3

import math
import os
import sys
import unittest

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    Shape,
)
from autoware_planning_msgs.msg import PathPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_dynamic_avoidance_tools import DynamicAvoidanceEngine, DynamicAvoidanceParameters
from bpp_path_tools import quaternion_from_yaw


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in range(121):
        point = PathPoint()
        point.pose.position.x = float(x)
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = 8.0
        item = PathPointWithLaneId()
        item.point = point
        item.lane_ids = [10]
        path.points.append(item)
    path.left_bound = [Point(x=0.0, y=2.5), Point(x=120.0, y=2.5)]
    path.right_bound = [Point(x=0.0, y=-2.5), Point(x=120.0, y=-2.5)]
    return path


def make_odometry(speed=0.0):
    message = Odometry()
    message.pose.pose.position.x = 5.0
    message.pose.pose.orientation = quaternion_from_yaw(0.0)
    message.twist.twist.linear.x = speed
    return message


def make_object(x=50.0, y=1.8, yaw=math.pi, speed=5.0, object_id=1):
    obj = PredictedObject()
    obj.object_id.uuid = [object_id] + [0] * 15
    classification = ObjectClassification()
    classification.label = ObjectClassification.CAR
    classification.probability = 1.0
    obj.classification.append(classification)
    obj.shape.type = Shape.BOUNDING_BOX
    obj.shape.dimensions.x = 4.5
    obj.shape.dimensions.y = 2.0
    obj.shape.dimensions.z = 1.6
    pose = obj.kinematics.initial_pose_with_covariance.pose
    pose.position.x = x
    pose.position.y = y
    pose.orientation = quaternion_from_yaw(yaw)
    obj.kinematics.initial_twist_with_covariance.twist.linear.x = speed
    return obj


def message(*objects):
    result = PredictedObjects()
    result.objects = list(objects)
    return result


class DynamicAvoidanceEngineTest(unittest.TestCase):
    def setUp(self):
        self.path = make_path()
        self.odometry = make_odometry()
        self.engine = DynamicAvoidanceEngine(DynamicAvoidanceParameters())

    def plan_frames(self, obj, count=5):
        result = None
        for frame in range(count):
            result = self.engine.plan(
                self.path, message(obj), self.odometry, float(frame) * 0.1
            )
        return result

    def test_oncoming_vehicle_cuts_left_bound_after_five_observations(self):
        for frame in range(4):
            result = self.engine.plan(
                self.path, message(make_object()), self.odometry, frame * 0.1
            )
            self.assertFalse(result.targets)
            self.assertEqual(
                result.candidates[0].failed_reason,
                "insufficient_successive_observations",
            )

        result = self.engine.plan(
            self.path, message(make_object()), self.odometry, 0.4
        )
        self.assertEqual(len(result.targets), 1)
        target = result.targets[0]
        self.assertEqual(target.collision_side, "left")
        self.assertLess(target.time_to_collision, 10.0)
        self.assertAlmostEqual(target.inner_bound_offset, 0.448, delta=0.05)
        self.assertAlmostEqual(min(point.y for point in result.output_path.left_bound), 0.448, delta=0.05)
        self.assertEqual(
            [item.point.pose.position.x for item in result.output_path.points],
            [item.point.pose.position.x for item in self.path.points],
        )
        self.assertEqual(
            [item.point.longitudinal_velocity_mps for item in result.output_path.points],
            [item.point.longitudinal_velocity_mps for item in self.path.points],
        )

    def test_front_vehicle_is_left_for_longitudinal_planner(self):
        result = self.plan_frames(make_object(y=0.0, yaw=0.0, speed=1.0))
        self.assertFalse(result.targets)
        self.assertEqual(result.candidates[0].failed_reason, "front_object_to_follow")

    def test_fast_crossing_vehicle_is_filtered_during_rough_stage(self):
        result = self.plan_frames(
            make_object(y=1.0, yaw=0.5 * math.pi, speed=2.0), count=1
        )
        self.assertFalse(result.targets)
        self.assertEqual(result.candidates[0].failed_reason, "crossing_object")

    def test_valid_object_is_held_for_exit_hysteresis(self):
        active = self.plan_frames(make_object())
        self.assertEqual(len(active.targets), 1)
        held = self.engine.plan(self.path, message(), self.odometry, 0.6)
        self.assertEqual(len(held.targets), 1)
        for frame in range(4):
            held = self.engine.plan(self.path, message(), self.odometry, 0.7 + frame * 0.1)
        self.assertFalse(held.targets)

    def test_multiple_objects_cut_both_bounds(self):
        result = None
        for frame in range(5):
            result = self.engine.plan(
                self.path,
                message(
                    make_object(y=1.8, object_id=1),
                    make_object(x=60.0, y=-1.8, object_id=2),
                ),
                self.odometry,
                frame * 0.1,
            )
        self.assertEqual(len(result.targets), 2)
        self.assertEqual(
            {target.collision_side for target in result.targets}, {"left", "right"}
        )
        self.assertLess(min(point.y for point in result.output_path.left_bound), 1.0)
        self.assertGreater(max(point.y for point in result.output_path.right_bound), -1.0)


if __name__ == "__main__":
    unittest.main()
