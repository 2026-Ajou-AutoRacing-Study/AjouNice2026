#!/usr/bin/env python3

import math
import os
import sys
from copy import deepcopy
from types import SimpleNamespace
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
    ShiftLine,
    StaticAvoidanceEngine,
    StaticAvoidanceParameters,
    StaticAvoidanceResult,
    StaticAvoidanceScene,
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

    def test_static_non_target_object_collision_is_unsafe(self):
        line = ShiftLine(
            object_ids=["target"],
            direction="left",
            shift_length=2.0,
            start_s=0.0,
            avoid_end_s=20.0,
            return_start_s=80.0,
            return_end_s=100.0,
        )
        candidate_path, _ = self.engine._shift_path(self.path, [line])
        static_other_object = make_object(
            x=50.0, y=3.2, speed=0.0, object_id=2
        )

        safe, reason = self.engine._raw_safety(
            candidate_path,
            targets=[],
            all_objects=[static_other_object],
            conflict=False,
        )

        self.assertFalse(safe)
        self.assertEqual(reason, "static_other_object_collision")

    def test_static_non_target_object_with_hard_margin_is_safe(self):
        line = ShiftLine(
            object_ids=["target"],
            direction="left",
            shift_length=2.0,
            start_s=0.0,
            avoid_end_s=20.0,
            return_start_s=80.0,
            return_end_s=100.0,
        )
        candidate_path, _ = self.engine._shift_path(self.path, [line])
        static_other_object = make_object(
            x=50.0, y=4.2, speed=0.0, object_id=2
        )

        safe, reason = self.engine._raw_safety(
            candidate_path,
            targets=[],
            all_objects=[static_other_object],
            conflict=False,
        )

        self.assertTrue(safe)
        self.assertEqual(reason, "safe")

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

    def test_shift_profile_is_shortened_before_narrow_corridors(self):
        line = ShiftLine(
            object_ids=["object"],
            direction="left",
            shift_length=1.0,
            start_s=0.0,
            avoid_end_s=50.0,
            return_start_s=50.0,
            return_end_s=110.0,
        )
        stations = [float(value) for value in range(121)]
        lower = [-2.0] * len(stations)
        upper = [
            0.2 if station < 30.0 or station > 75.0 else 2.0
            for station in stations
        ]

        fitted = self.engine._fit_shift_line_to_bounds(
            line, stations, lower, upper, ego_speed=4.0
        )

        self.assertTrue(fitted)
        self.assertGreater(line.start_s, 0.0)
        self.assertLess(line.return_end_s, 110.0)
        self.assertTrue(
            self.engine._line_is_within_bounds(line, stations, lower, upper)
        )

    def test_return_distance_uses_return_nominal_speed_only(self):
        parameters = make_parameters()
        parameters.nominal_avoidance_speed = 8.33
        parameters.nominal_return_speed = 4.17
        parameters.return_min_jerk = 0.6
        engine = StaticAvoidanceEngine(parameters, make_map())
        objects = PredictedObjects()
        objects.objects = [make_object()]

        engine.plan(self.path, objects, self.odometry, 0.0)
        result = engine.plan(self.path, objects, self.odometry, 2.1)

        self.assertEqual(len(result.shift_lines), 1)
        line = result.shift_lines[0]
        return_distance = line.return_end_s - line.return_start_s
        expected = 4.0 * parameters.nominal_return_speed * (
            abs(line.shift_length) / (2.0 * parameters.return_min_jerk)
        ) ** (1.0 / 3.0)
        self.assertAlmostEqual(return_distance, expected, delta=2.0)
        self.assertLess(return_distance, 25.0)

    def test_fully_passed_object_does_not_insert_current_position_stop(self):
        target = SimpleNamespace(
            longitudinal=0.0,
            relative_longitudinal=-10.0,
            half_longitudinal=2.0,
        )

        output, stop_distance = self.engine._stop_path(
            self.path, ego_s=5.0, nearest_target=target
        )

        self.assertEqual(stop_distance, 0.0)
        self.assertTrue(
            all(
                point.point.longitudinal_velocity_mps > 4.0
                for point in output.points
            )
        )

    def test_registered_return_is_kept_until_actual_shift_is_recovered(self):
        class FakeRTCController:
            def __init__(self):
                self.completed = 0

            def evaluate(self, *_args, **_kwargs):
                return True, 0, None, True

            def complete(self, _stamp):
                self.completed += 1

        line = ShiftLine(
            object_ids=["object"],
            direction="left",
            shift_length=1.0,
            start_s=0.0,
            avoid_end_s=10.0,
            return_start_s=20.0,
            return_end_s=40.0,
        )
        registered_path, _ = self.engine._shift_path(self.path, [line])
        scene = StaticAvoidanceScene.__new__(StaticAvoidanceScene)
        scene.params = make_parameters()
        scene.engine = self.engine
        scene.avoid_rtc_controller = FakeRTCController()
        scene.return_rtc_controller = FakeRTCController()
        scene._return_in_progress = False
        scene._registered_return_path = None
        scene._registered_return_reference = None
        scene._registered_return_stations = []
        scene._registered_return_offsets = []
        scene._registered_avoid_direction = None
        scene._registered_return_direction = None
        scene._registered_avoid_signature = None
        scene._registered_return_signature = None
        scene._registered_max_shift = 0.0
        seed = StaticAvoidanceResult(
            reference_path=self.path,
            candidate_path=registered_path,
            output_path=registered_path,
            candidates=[],
            targets=[],
            shift_lines=[line],
            direction="left",
            return_direction="right",
            max_shift=1.0,
            signature=(("object", "left", 1.0),),
        )
        return_signature = seed.signature + (("return", "right"),)
        scene._register_return(seed, registered_path, return_signature)

        odometry = make_odometry()
        odometry.pose.pose.position.x = 25.0
        odometry.pose.pose.position.y = 0.8
        result = StaticAvoidanceResult(
            reference_path=self.path,
            candidate_path=deepcopy(self.path),
            output_path=deepcopy(self.path),
            candidates=[],
            targets=[],
            shift_lines=[],
            valid=False,
        )

        active = scene._continue_registered_return(result, odometry, 1.0)

        self.assertTrue(active)
        self.assertTrue(scene._return_in_progress)
        self.assertGreater(result.output_path.points[25].point.pose.position.y, 0.1)
        self.assertEqual(scene.avoid_rtc_controller.completed, 0)

        odometry.pose.pose.position.x = 50.0
        odometry.pose.pose.position.y = 0.0
        completed = scene._continue_registered_return(result, odometry, 2.0)

        self.assertFalse(completed)
        self.assertFalse(scene._return_in_progress)
        self.assertEqual(scene.avoid_rtc_controller.completed, 1)
        self.assertEqual(scene.return_rtc_controller.completed, 1)


if __name__ == "__main__":
    unittest.main()
