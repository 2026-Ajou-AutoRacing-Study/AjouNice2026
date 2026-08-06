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

from bpp_map_tools import CenterlineMap
from bpp_path_tools import quaternion_from_yaw
from bpp_static_avoidance_tools import (
    ObjectPolicy,
    StaticAvoidanceEngine,
    StaticAvoidanceParameters,
)


def make_map():
    centerlines = {
        "10": [(0.0, 0.0, 0.0), (120.0, 0.0, 0.0)],
        "11": [(0.0, 4.5, 0.0), (120.0, 4.5, 0.0)],
    }
    boundaries = {
        "10": (
            [(0.0, 2.5, 0.0), (120.0, 2.5, 0.0)],
            [(0.0, -2.5, 0.0), (120.0, -2.5, 0.0)],
        ),
        "11": (
            [(0.0, 6.5, 0.0), (120.0, 6.5, 0.0)],
            [(0.0, 2.5, 0.0), (120.0, 2.5, 0.0)],
        ),
    }
    boundary_ids = {"10": (100, 101), "11": (102, 100)}
    return CenterlineMap("synthetic", centerlines, boundaries, boundary_ids)


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in range(121):
        point = PathPoint()
        point.pose.position.x = float(x)
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = 4.1666667
        item = PathPointWithLaneId()
        item.point = point
        item.lane_ids = [10]
        path.points.append(item)
    for x in (0.0, 120.0):
        left = Point(x=x, y=2.5, z=0.0)
        right = Point(x=x, y=-2.5, z=0.0)
        path.left_bound.append(left)
        path.right_bound.append(right)
    return path


def make_odometry():
    message = Odometry()
    message.pose.pose.position.x = 5.0
    message.pose.pose.orientation = quaternion_from_yaw(0.0)
    return message


def make_object(x=50.0, y=-1.8, speed=0.0, object_id=1):
    predicted = PredictedObject()
    predicted.object_id.uuid = [object_id] + [0] * 15
    classification = ObjectClassification()
    classification.label = ObjectClassification.CAR
    classification.probability = 1.0
    predicted.classification.append(classification)
    predicted.shape.type = Shape.BOUNDING_BOX
    predicted.shape.dimensions.x = 4.5
    predicted.shape.dimensions.y = 2.0
    predicted.shape.dimensions.z = 1.6
    predicted.kinematics.initial_pose_with_covariance.pose.position.x = x
    predicted.kinematics.initial_pose_with_covariance.pose.position.y = y
    predicted.kinematics.initial_pose_with_covariance.pose.orientation = quaternion_from_yaw(0.0)
    predicted.kinematics.initial_twist_with_covariance.twist.linear.x = speed
    return predicted


def make_parameters():
    params = StaticAvoidanceParameters()
    params.target_labels = {ObjectClassification.CAR}
    params.object_policies = {
        ObjectClassification.CAR: ObjectPolicy(1.0, 2.0, 0.5, 0.2, 0.2, 0.0, 0.1),
        ObjectClassification.UNKNOWN: ObjectPolicy(
            0.28, 1.0, 0.5, -0.2, -0.2, 0.0, 0.1
        ),
    }
    return params


class StaticAvoidanceEngineTest(unittest.TestCase):
    def setUp(self):
        self.path = make_path()
        self.odometry = make_odometry()
        self.engine = StaticAvoidanceEngine(make_parameters(), make_map())

    def plan(self, objects, stamp):
        message = PredictedObjects()
        message.objects = list(objects)
        return self.engine.plan(self.path, message, self.odometry, stamp)

    def test_requires_stopped_object_history_then_generates_left_shift(self):
        first = self.plan([make_object()], 0.0)
        self.assertEqual(first.candidates[0].failed_reason, "unstable_stopped_object")

        result = self.plan([make_object()], 2.1)
        self.assertEqual(len(result.targets), 1)
        self.assertEqual(result.direction, "left")
        self.assertTrue(result.targets[0].is_parked)
        self.assertAlmostEqual(result.max_shift, 0.9, delta=0.2)
        self.assertTrue(result.safe)
        self.assertTrue(result.comfortable)
        self.assertTrue(result.valid)
        self.assertLess(result.start_distance, 8.0)
        self.assertAlmostEqual(result.finish_distance, 48.5, delta=2.0)
        result.activated = True
        self.assertFalse(self.engine.turn_signal_required(result))

    def test_filters_moving_object(self):
        result = self.plan([make_object(speed=2.0)], 3.0)
        self.assertFalse(result.targets)
        self.assertEqual(result.candidates[0].failed_reason, "moving_object")

    def test_merges_overlapping_objects_on_same_side(self):
        self.plan([make_object(), make_object(x=53.0, object_id=2)], 0.0)
        result = self.plan(
            [make_object(), make_object(x=53.0, object_id=2)], 2.1
        )
        self.assertEqual(len(result.targets), 2)
        self.assertEqual(len(result.shift_lines), 1)
        self.assertEqual(result.shift_lines[0].direction, "left")

    def test_intersection_area_is_propagated_to_output_drivable_bounds(self):
        centerline_map = make_map()
        centerline_map.intersection_areas = {
            "500": [
                (20.0, -5.0, 0.0),
                (80.0, -5.0, 0.0),
                (80.0, 5.0, 0.0),
                (20.0, 5.0, 0.0),
                (20.0, -5.0, 0.0),
            ]
        }
        centerline_map.lane_intersection_area_ids = {"10": "500"}
        parameters = make_parameters()
        parameters.use_lane_type = "current_lane"
        engine = StaticAvoidanceEngine(parameters, centerline_map)

        output, _ = engine._shift_path(self.path, [])

        self.assertAlmostEqual(output.left_bound[10].y, 2.5)
        self.assertAlmostEqual(output.right_bound[10].y, -2.5)
        self.assertAlmostEqual(output.left_bound[50].y, 5.0)
        self.assertAlmostEqual(output.right_bound[50].y, -5.0)
        self.assertAlmostEqual(self.path.left_bound[0].y, 2.5)
        self.assertEqual(len(self.path.left_bound), 2)


if __name__ == "__main__":
    unittest.main()
