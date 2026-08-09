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
from bpp_path_tools import project_pose_to_path, quaternion_from_yaw
from bpp_static_avoidance_tools import (
    AvoidanceObject,
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
        line = result.shift_lines[0]
        target = result.targets[0]
        self.assertLess(line.avoid_end_s, target.longitudinal)
        self.assertGreater(line.return_start_s, target.longitudinal)
        self.assertGreater(line.return_start_s - line.avoid_end_s, 5.0)
        self.assertAlmostEqual(
            line.value_at(target.longitudinal), line.shift_length, delta=1.0e-6
        )
        result.activated = True
        self.assertFalse(self.engine.turn_signal_required(result))

    def test_uses_other_side_when_primary_direction_has_insufficient_space(self):
        parameters = make_parameters()
        parameters.object_policies[ObjectClassification.UNKNOWN] = ObjectPolicy(
            0.28, 1.0, 0.4, 0.2, 0.7, 0.0, 0.1
        )
        engine = StaticAvoidanceEngine(parameters, make_map())
        candidate = AvoidanceObject(
            object_id="unknown",
            message=make_object(),
            label=ObjectClassification.UNKNOWN,
            longitudinal=50.0,
            relative_longitudinal=45.0,
            lateral=0.70,
            half_longitudinal=0.30,
            half_lateral=1.06,
            current_left_bound=1.69,
            current_right_bound=1.68,
            expanded_left_bound=5.60,
            expanded_right_bound=2.14,
            stopped_time=5.0,
            total_movement=0.0,
            avoid_required=True,
        )

        engine._desired_shift(candidate, ego_speed=2.7, path_end=120.0)

        self.assertTrue(candidate.allow_avoidance)
        self.assertEqual(candidate.direction, "left")
        self.assertAlmostEqual(candidate.shift_length, 3.3, delta=0.1)
        self.assertGreater(candidate.available_shift, 4.3)
        self.assertGreaterEqual(candidate.start_s, 6.0)

    def test_bound_distance_uses_exact_object_projection_on_coarse_path(self):
        class PositionSensitiveMap:
            def __init__(self):
                self.last_position = None

            @staticmethod
            def has_lane(_lane_id):
                return True

            def expanded_bound_distances(self, _lane_id, position, *_args):
                self.last_position = position
                return 5.0, float(position[0])

        path = PathWithLaneId()
        for x in (0.0, 10.0):
            point = PathPoint()
            point.pose.position.x = x
            point.pose.orientation = quaternion_from_yaw(0.0)
            item = PathPointWithLaneId()
            item.point = point
            item.lane_ids = [10]
            path.points.append(item)
        path.left_bound = [Point(x=0.0, y=2.5), Point(x=10.0, y=2.5)]
        path.right_bound = [Point(x=0.0, y=-2.5), Point(x=10.0, y=-2.5)]

        object_message = make_object(x=8.0, y=0.7)
        projection = project_pose_to_path(
            path,
            object_message.kinematics.initial_pose_with_covariance.pose,
        )
        centerline_map = PositionSensitiveMap()
        engine = StaticAvoidanceEngine(make_parameters(), centerline_map)

        bounds = engine._bound_distances(path, projection)

        self.assertEqual(projection.index, 0)
        self.assertAlmostEqual(projection.segment_ratio, 0.8)
        self.assertAlmostEqual(centerline_map.last_position[0], 8.0)
        self.assertAlmostEqual(bounds[3], 8.0)

    def test_relaxes_soft_drivable_bound_to_hard_bound_when_needed(self):
        parameters = make_parameters()
        parameters.object_policies[ObjectClassification.UNKNOWN] = ObjectPolicy(
            0.28, 1.0, 0.4, 0.2, 0.7, 0.0, 0.1
        )
        parameters.soft_drivable_bound_margin = 0.3
        parameters.hard_drivable_bound_margin = 0.0
        engine = StaticAvoidanceEngine(parameters, make_map())
        candidate = AvoidanceObject(
            object_id="unknown",
            message=make_object(),
            label=ObjectClassification.UNKNOWN,
            longitudinal=50.0,
            relative_longitudinal=45.0,
            lateral=0.70,
            half_longitudinal=0.30,
            half_lateral=1.06,
            current_left_bound=1.69,
            current_right_bound=1.68,
            expanded_left_bound=2.14,
            expanded_right_bound=2.49,
            stopped_time=5.0,
            total_movement=0.0,
            avoid_required=True,
        )
        stations = [float(value) for value in range(121)]
        lower = [-1.55] * len(stations)
        upper = [5.0] * len(stations)
        expanded_path = make_path()

        trial, line, failure = engine._fit_directional_candidate(
            candidate,
            "right",
            2.7,
            120.0,
            expanded_path,
            stations,
            lower,
            upper,
        )

        self.assertEqual(failure, "")
        self.assertIsNotNone(trial)
        self.assertTrue(trial.uses_hard_drivable_bound)
        self.assertTrue(line.uses_hard_drivable_bound)
        self.assertAlmostEqual(trial.shift_length, -1.5, delta=0.05)
        self.assertAlmostEqual(trial.available_shift, -1.54, delta=0.02)

    def test_profile_bound_uses_hard_margin_minus_maximum_deviation(self):
        parameters = make_parameters()
        parameters.vehicle_width = 2.0
        parameters.hard_drivable_bound_margin = 0.3
        parameters.max_deviation_from_lane = 0.2
        engine = StaticAvoidanceEngine(parameters)

        _expanded, _stations, lower, upper = engine._expanded_profile_bounds(
            make_path()
        )

        # 2.5 m lane half-width - (1.0 m vehicle half-width + 0.3 m
        # hard margin - 0.2 m allowed deviation) = 1.4 m center shift.
        self.assertAlmostEqual(lower[0], -1.4)
        self.assertAlmostEqual(upper[0], 1.4)

    def test_diagonal_profile_relaxes_shift_and_delays_completion(self):
        engine = StaticAvoidanceEngine(make_parameters())
        candidate = AvoidanceObject(
            object_id="diagonal",
            message=make_object(x=24.0, y=0.7),
            label=ObjectClassification.UNKNOWN,
            longitudinal=24.0,
            relative_longitudinal=19.0,
            lateral=0.7,
            half_longitudinal=0.3,
            half_lateral=1.06,
            current_left_bound=2.5,
            current_right_bound=2.5,
            expanded_left_bound=5.0,
            expanded_right_bound=3.0,
            stopped_time=5.0,
            total_movement=0.0,
            avoid_required=True,
            allow_avoidance=True,
            shift_length=-1.9,
            minimum_shift_length=-1.5,
            start_s=10.0,
            avoid_end_s=20.0,
            return_start_s=30.0,
            return_end_s=45.0,
        )
        line = ShiftLine(
            object_ids=[candidate.object_id],
            direction="right",
            shift_length=-1.9,
            start_s=10.0,
            avoid_end_s=20.0,
            return_start_s=30.0,
            return_end_s=45.0,
        )
        stations = [float(value) for value in range(121)]
        lower = [-1.5] * len(stations)
        upper = [5.0] * len(stations)
        lower[18] = -1.0
        lower[19] = -1.2
        lower[20] = -1.35
        lower[21] = -1.45
        engine._profile_footprint_clearance = lambda *_args, **_kwargs: (
            True,
            "safe",
            0.2,
            0.3,
            18.0,
        )

        fitted, failure, metrics = engine._fit_diagonal_shift_line(
            candidate,
            line,
            make_path(),
            stations,
            lower,
            upper,
            ego_speed=1.38,
        )

        self.assertEqual(failure, "")
        self.assertIsNotNone(fitted)
        self.assertTrue(fitted.uses_diagonal_profile)
        self.assertGreater(fitted.avoid_end_s, line.avoid_end_s)
        self.assertLess(abs(fitted.shift_length), abs(line.shift_length))
        self.assertGreaterEqual(abs(fitted.shift_length), 1.5)
        self.assertEqual(metrics[1], "safe")

    def test_prefers_shorter_conventional_side_when_both_sides_are_feasible(self):
        parameters = make_parameters()
        parameters.object_policies[ObjectClassification.UNKNOWN] = ObjectPolicy(
            0.28, 1.0, 0.4, 0.2, 0.7, 0.0, 0.1
        )
        engine = StaticAvoidanceEngine(parameters, make_map())
        candidate = AvoidanceObject(
            object_id="unknown",
            message=make_object(),
            label=ObjectClassification.UNKNOWN,
            longitudinal=50.0,
            relative_longitudinal=45.0,
            lateral=0.70,
            half_longitudinal=0.30,
            half_lateral=1.06,
            current_left_bound=1.69,
            current_right_bound=1.68,
            expanded_left_bound=5.60,
            expanded_right_bound=5.60,
            stopped_time=5.0,
            total_movement=0.0,
            avoid_required=True,
        )

        engine._desired_shift(candidate, ego_speed=2.7, path_end=120.0)

        self.assertTrue(candidate.allow_avoidance)
        self.assertEqual(candidate.direction, "right")
        self.assertAlmostEqual(candidate.shift_length, -1.9, delta=0.1)

    def test_rejects_avoidance_when_neither_side_keeps_hard_margin(self):
        parameters = make_parameters()
        parameters.object_policies[ObjectClassification.UNKNOWN] = ObjectPolicy(
            0.28, 1.0, 0.4, 0.2, 0.7, 0.0, 0.1
        )
        engine = StaticAvoidanceEngine(parameters, make_map())
        candidate = AvoidanceObject(
            object_id="unknown",
            message=make_object(),
            label=ObjectClassification.UNKNOWN,
            longitudinal=50.0,
            relative_longitudinal=45.0,
            lateral=0.70,
            half_longitudinal=0.30,
            half_lateral=1.06,
            current_left_bound=1.69,
            current_right_bound=1.68,
            expanded_left_bound=2.14,
            expanded_right_bound=2.14,
            stopped_time=5.0,
            total_movement=0.0,
            avoid_required=True,
        )

        engine._desired_shift(candidate, ego_speed=2.7, path_end=120.0)

        self.assertFalse(candidate.allow_avoidance)
        self.assertEqual(candidate.failed_reason, "insufficient_drivable_space")

    def test_plan_uses_alternate_only_after_primary_full_profile_fails(self):
        parameters = make_parameters()
        parameters.enable_safety_check = False
        parameters.ambiguous_stopped_time = 2.0
        parameters.ambiguous_policy = "auto"
        engine = StaticAvoidanceEngine(parameters)
        wide_path = make_path()
        wide_path.left_bound = [
            Point(x=0.0, y=6.0),
            Point(x=120.0, y=6.0),
        ]
        wide_path.right_bound = [
            Point(x=0.0, y=-6.0),
            Point(x=120.0, y=-6.0),
        ]
        objects = PredictedObjects()
        objects.objects = [make_object()]
        engine.plan(wide_path, objects, self.odometry, 0.0)

        original_fit = engine._fit_directional_candidate
        attempted_directions = []

        def fail_primary(candidate, direction, *args):
            attempted_directions.append(direction)
            if direction == "left":
                return None, None, "shift_profile_outside_drivable_bounds"
            return original_fit(candidate, direction, *args)

        engine._fit_directional_candidate = fail_primary

        result = engine.plan(wide_path, objects, self.odometry, 2.1)

        self.assertEqual(attempted_directions, ["left", "right"])
        self.assertEqual(len(result.targets), 1)
        self.assertEqual(result.direction, "right")
        self.assertTrue(result.valid)
        self.assertIn("alternate_direction_fallback", result.status)
        self.assertIn(
            "shift_profile_outside_drivable_bounds", result.status
        )

    def test_plan_retries_alternate_when_primary_path_is_unsafe(self):
        parameters = make_parameters()
        parameters.ambiguous_stopped_time = 2.0
        parameters.ambiguous_policy = "auto"
        engine = StaticAvoidanceEngine(parameters)
        wide_path = make_path()
        wide_path.left_bound = [
            Point(x=0.0, y=6.0),
            Point(x=120.0, y=6.0),
        ]
        wide_path.right_bound = [
            Point(x=0.0, y=-6.0),
            Point(x=120.0, y=-6.0),
        ]
        objects = PredictedObjects()
        objects.objects = [make_object()]
        engine.plan(wide_path, objects, self.odometry, 0.0)
        checked_directions = []

        def primary_is_unsafe(_path, targets, _objects, _conflict):
            checked_directions.append(targets[0].direction)
            if targets[0].direction == "left":
                return False, "forced_primary_collision"
            return True, "safe"

        engine._raw_safety = primary_is_unsafe

        result = engine.plan(wide_path, objects, self.odometry, 2.1)

        self.assertEqual(checked_directions, ["left", "right"])
        self.assertEqual(result.direction, "right")
        self.assertTrue(result.safe)
        self.assertTrue(result.valid)
        self.assertIn("forced_primary_collision", result.status)

    def test_plan_stops_when_primary_and_alternate_profiles_fail(self):
        parameters = make_parameters()
        engine = StaticAvoidanceEngine(parameters)
        objects = PredictedObjects()
        objects.objects = [make_object()]
        engine.plan(self.path, objects, self.odometry, 0.0)
        attempted_directions = []

        def fail_both(_candidate, direction, *_args):
            attempted_directions.append(direction)
            return None, None, "shift_profile_outside_drivable_bounds"

        engine._fit_directional_candidate = fail_both

        result = engine.plan(self.path, objects, self.odometry, 2.1)

        self.assertEqual(attempted_directions, ["left", "right"])
        self.assertFalse(result.targets)
        self.assertFalse(result.valid)
        self.assertGreater(result.stop_distance, 0.0)
        self.assertIn("alternate_right", result.status)

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

    def test_activated_avoidance_keeps_registered_shift_until_return(self):
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
            avoid_end_s=20.0,
            return_start_s=60.0,
            return_end_s=90.0,
        )
        full_candidate, _ = self.engine._shift_path(self.path, [line])
        avoid_only_line = deepcopy(line)
        avoid_only_line.return_start_s = 121.0
        avoid_only_line.return_end_s = 122.0
        avoid_candidate, _ = self.engine._shift_path(
            self.path, [avoid_only_line]
        )

        scene = StaticAvoidanceScene.__new__(StaticAvoidanceScene)
        scene.params = make_parameters()
        scene.engine = self.engine
        scene.avoid_rtc_controller = FakeRTCController()
        scene.return_rtc_controller = FakeRTCController()
        scene._return_in_progress = False
        seed = StaticAvoidanceResult(
            reference_path=self.path,
            candidate_path=avoid_candidate,
            output_path=avoid_candidate,
            candidates=[],
            targets=[],
            shift_lines=[line],
            direction="left",
            return_direction="right",
            max_shift=1.0,
            signature=(("object", "left", 1.0),),
        )
        scene._register_avoidance(seed, avoid_candidate, full_candidate)

        # Simulate a following cycle where fresh planning no longer emits a
        # valid shift line.  The activated maneuver must not snap back to the
        # new centerline before its separately approved return phase.
        fresh_result = StaticAvoidanceResult(
            reference_path=self.path,
            candidate_path=deepcopy(self.path),
            output_path=deepcopy(self.path),
            candidates=[],
            targets=[],
            shift_lines=[],
            valid=False,
        )
        objects = PredictedObjects()

        active = scene._continue_committed_avoidance(
            fresh_result, objects, self.odometry, 1.0
        )

        self.assertTrue(active)
        self.assertTrue(scene._avoid_in_progress)
        self.assertTrue(fresh_result.activated)
        self.assertAlmostEqual(
            fresh_result.output_path.points[30].point.pose.position.y,
            1.0,
            delta=0.05,
        )
        self.assertAlmostEqual(
            fresh_result.output_path.points[70].point.pose.position.y,
            1.0,
            delta=0.05,
        )
        self.assertLess(
            fresh_result.candidate_path.points[70].point.pose.position.y,
            1.0,
        )
        self.assertFalse(fresh_result.return_ready)

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
        scene._register_return(
            seed, registered_path, return_signature, self.odometry
        )

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

    def test_active_avoidance_limits_acceleration_and_holds_return_speed(self):
        scene = StaticAvoidanceScene.__new__(StaticAvoidanceScene)
        scene.params = make_parameters()
        scene.params.max_acceleration = 0.5
        scene.params.nominal_return_speed = 2.7
        odometry = make_odometry()
        odometry.twist.twist.linear.x = 2.0

        constrained = scene._apply_longitudinal_constraints(
            self.path,
            self.path,
            odometry,
            maneuver_end_s=40.0,
            return_start_s=20.0,
            return_speed_limit=2.0,
        )

        self.assertAlmostEqual(
            constrained.points[5].point.longitudinal_velocity_mps, 2.0
        )
        self.assertLessEqual(
            constrained.points[6].point.longitudinal_velocity_mps,
            math.sqrt(5.0) + 1.0e-6,
        )
        self.assertAlmostEqual(
            constrained.points[20].point.longitudinal_velocity_mps, 2.0
        )
        self.assertGreater(
            constrained.points[41].point.longitudinal_velocity_mps, 4.0
        )

    def test_registered_return_extends_when_actual_speed_exceeds_plan(self):
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
        scene._return_in_progress = False
        scene._registered_return_signature = None
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
        scene._register_return(
            seed, registered_path, return_signature, self.odometry
        )
        original_end = scene._registered_return_end_s

        fast_odometry = make_odometry()
        fast_odometry.pose.pose.position.x = 20.0
        fast_odometry.twist.twist.linear.x = 10.0
        extended = scene._extend_registered_return_for_speed(fast_odometry)

        self.assertTrue(extended)
        self.assertGreater(scene._registered_return_end_s, original_end)
        self.assertGreater(abs(scene._interpolate_registered_shift(40.0)), 0.05)

    def test_registered_return_reanchors_from_actual_lateral_offset(self):
        class FakeRTCController:
            def evaluate(self, *_args, **_kwargs):
                return True, 0, None, True

            def complete(self, _stamp):
                pass

        parameters = make_parameters()
        parameters.nominal_return_speed = 2.7
        parameters.return_min_jerk = 0.3
        parameters.return_reanchor_lateral_error_threshold = 0.3
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
        scene.params = parameters
        scene.engine = self.engine
        scene.avoid_rtc_controller = FakeRTCController()
        scene.return_rtc_controller = FakeRTCController()
        scene._return_in_progress = False
        scene._registered_return_signature = None
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
        scene._register_return(
            seed,
            registered_path,
            return_signature,
            self.odometry,
        )

        lagging_odometry = make_odometry()
        lagging_odometry.pose.pose.position.x = 34.0
        lagging_odometry.pose.pose.position.y = 0.9
        lagging_odometry.twist.twist.linear.x = 2.7
        result = StaticAvoidanceResult(
            reference_path=self.path,
            candidate_path=deepcopy(self.path),
            output_path=deepcopy(self.path),
            candidates=[],
            targets=[],
            shift_lines=[],
            valid=False,
        )

        active = scene._continue_registered_return(
            result,
            lagging_odometry,
            1.0,
        )

        self.assertTrue(active)
        self.assertIn("reanchored=True", result.status)
        self.assertAlmostEqual(scene._registered_return_start_s, 34.0, delta=0.1)
        self.assertGreater(scene._registered_return_end_s, 45.0)
        self.assertAlmostEqual(
            result.output_path.points[34].point.pose.position.y,
            0.9,
            delta=0.05,
        )

        recovered_odometry = make_odometry()
        recovered_odometry.pose.pose.position.x = 36.0
        recovered_odometry.pose.pose.position.y = 0.0
        recovered_result = StaticAvoidanceResult(
            reference_path=self.path,
            candidate_path=deepcopy(self.path),
            output_path=deepcopy(self.path),
            candidates=[],
            targets=[],
            shift_lines=[],
            valid=False,
        )

        active = scene._continue_registered_return(
            recovered_result,
            recovered_odometry,
            2.0,
        )

        self.assertFalse(active)
        self.assertFalse(scene._return_in_progress)


if __name__ == "__main__":
    unittest.main()
