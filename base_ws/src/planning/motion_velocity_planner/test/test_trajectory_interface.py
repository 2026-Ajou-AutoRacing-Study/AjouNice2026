#!/usr/bin/env python3

import os
import sys
import unittest
from copy import deepcopy
from unittest import mock

from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
)
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

import motion_velocity_planner as motion_velocity_planner_module
from motion_velocity_planner import MotionVelocityPlanner
from mvp_utils import (
    crop_trajectory_around_ego,
    path_cumulative_lengths,
    project_pose_to_path,
    set_velocity_window,
)


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


def predicted_object(x, y=0.0, label=ObjectClassification.UNKNOWN):
    obj = PredictedObject()
    obj.kinematics.initial_pose_with_covariance.pose.position.x = float(x)
    obj.kinematics.initial_pose_with_covariance.pose.position.y = float(y)
    obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
    classification = ObjectClassification()
    classification.label = label
    classification.probability = 1.0
    obj.classification.append(classification)
    return obj


def obstacle_planner():
    planner = MotionVelocityPlanner.__new__(MotionVelocityPlanner)
    planner.trajectory = trajectory([5.0] * 101)
    planner.objects = PredictedObjects()
    planner.objects.objects = [
        predicted_object(30.0, label=ObjectClassification.PEDESTRIAN),
        predicted_object(40.0, y=5.0),
    ]
    planner.odometry = Odometry()
    planner.odometry.pose.pose.orientation.w = 1.0
    planner.odometry.twist.twist.linear.x = 5.0
    planner.delta_yaw_threshold = 1.0472
    planner.max_lateral_distance = 1.5
    planner.min_classification_confidence = 0.5
    planner.stop_unknown_objects = False
    planner.stop_object_labels = ["pedestrian"]
    planner.stop_object_label_ids = {ObjectClassification.PEDESTRIAN}
    planner.dynamic_speed_threshold = 0.1
    planner.slow_dynamic_stop_speed_threshold = 1.0
    planner.dynamic_stop_min_relative_speed = 1.0
    planner.dynamic_stop_ttc_threshold = 4.0
    planner.default_velocity_mps = 5.0
    planner.use_velocity_limit = False
    planner.velocity_limit = None
    planner.reaction_time = 0.8
    planner.comfortable_deceleration = 3.0
    planner.object_size_margin = 0.0
    planner.dynamic_object_time_margin = 1.0
    planner.stop_distance = 3.0
    planner.dynamic_stop_distance = 5.0
    planner.warning_distance = 6.0
    planner.min_stop_velocity = 0.5
    planner.max_deceleration = 2.0
    planner.stop_slow_distance = 3.0
    planner.slowdown_distance = 15.0
    planner.min_slow_velocity = 1.0
    planner.cruise_speed = 5.0
    planner.safe_distance = 10.0
    planner.distance_buffer = 0.0
    planner.min_ttc = 1.0
    planner.min_adjusted_velocity = 2.5
    return planner


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

    def test_projection_preserves_negative_station_behind_path_start(self):
        message = trajectory([5.0] * 5)
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = -100.0
        pose.position.y = 0.4
        projection = project_pose_to_path(message, pose)
        self.assertAlmostEqual(projection.longitudinal_distance, -100.0)
        self.assertAlmostEqual(projection.signed_lateral_distance, 0.4)
        self.assertAlmostEqual(projection.distance, 100.0007999968)

    def test_velocity_window_supports_native_trajectory(self):
        output = set_velocity_window(trajectory([5.0] * 5), 1.0, 3.0, 2.0)
        self.assertEqual(
            [point.longitudinal_velocity_mps for point in output.points],
            [5.0, 2.0, 2.0, 2.0, 5.0],
        )

    def test_crop_starts_at_ego_projection_and_limits_forward_length(self):
        message = trajectory([5.0] * 301)
        pose = Pose()
        pose.position.x = 80.25
        pose.orientation.w = 1.0
        output = crop_trajectory_around_ego(
            message, pose, forward_length=120.0, backward_length=0.0
        )
        self.assertAlmostEqual(output.points[0].pose.position.x, 80.25)
        self.assertAlmostEqual(output.points[-1].pose.position.x, 200.25)
        self.assertAlmostEqual(path_cumulative_lengths(output)[-1], 120.0)

    def test_yaw_constrained_projection_ignores_reverse_parallel_segment(self):
        message = Trajectory()
        for x, y, yaw in (
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (10.0, 0.2, 3.141592653589793),
            (0.0, 0.2, 3.141592653589793),
        ):
            point = TrajectoryPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.orientation.z = __import__("math").sin(yaw * 0.5)
            point.pose.orientation.w = __import__("math").cos(yaw * 0.5)
            message.points.append(point)
        pose = Pose()
        pose.position.x = 5.0
        pose.position.y = 0.19
        pose.orientation.w = 1.0
        projection = project_pose_to_path(message, pose, yaw_threshold=1.0)
        self.assertEqual(projection.index, 0)
        self.assertAlmostEqual(projection.longitudinal_distance, 5.0)

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

    def test_stop_projection_rejects_object_behind_ego(self):
        planner = MotionVelocityPlanner.__new__(MotionVelocityPlanner)
        planner.trajectory = trajectory([5.0] * 31)
        planner.odometry = Odometry()
        planner.odometry.pose.pose.position.x = 10.0
        planner.odometry.pose.pose.orientation.w = 1.0
        planner.delta_yaw_threshold = 1.0472
        planner.max_lateral_distance = 1.5
        planner.min_classification_confidence = 0.5
        planner.stop_unknown_objects = True
        planner.stop_object_labels = []
        planner.stop_object_label_ids = set()

        self.assertIsNone(planner.stop_object_projection(predicted_object(5.0, 0.2)))
        projection = planner.stop_object_projection(predicted_object(20.0, 0.2))
        self.assertIsNotNone(projection)
        self.assertAlmostEqual(projection.longitudinal_distance, 20.0)

    def test_shared_projection_cache_preserves_velocity_result(self):
        planner = obstacle_planner()
        distances = path_cumulative_lengths(planner.trajectory)
        cached_output = deepcopy(planner.trajectory)
        uncached_output = deepcopy(planner.trajectory)

        cache = planner.build_object_projection_cache(
            planner.trajectory, planner.objects, planner.odometry
        )
        planner.apply_stop(cached_output, distances, cache)
        planner.apply_dynamic_stop(cached_output, distances, cache)
        planner.apply_slowdown(cached_output, distances, cache)
        planner.apply_cruise(cached_output, distances, cache)
        planner.apply_ttc_limit(cached_output, distances, cache)

        planner.apply_stop(uncached_output, distances)
        planner.apply_dynamic_stop(uncached_output, distances)
        planner.apply_slowdown(uncached_output, distances)
        planner.apply_cruise(uncached_output, distances)
        planner.apply_ttc_limit(uncached_output, distances)

        self.assertEqual(
            [point.longitudinal_velocity_mps for point in cached_output.points],
            [point.longitudinal_velocity_mps for point in uncached_output.points],
        )

    def test_shared_projection_cache_avoids_reprojection_in_velocity_steps(self):
        planner = obstacle_planner()
        output = deepcopy(planner.trajectory)
        distances = path_cumulative_lengths(output)

        with mock.patch.object(
            motion_velocity_planner_module,
            "project_pose_to_path",
            wraps=project_pose_to_path,
        ) as projection:
            cache = planner.build_object_projection_cache(
                planner.trajectory, planner.objects, planner.odometry
            )
            calls_after_cache_build = projection.call_count
            planner.apply_stop(output, distances, cache)
            planner.apply_dynamic_stop(output, distances, cache)
            planner.apply_slowdown(output, distances, cache)
            planner.apply_cruise(output, distances, cache)
            planner.apply_ttc_limit(output, distances, cache)

        self.assertEqual(calls_after_cache_build, 3)
        self.assertEqual(projection.call_count, calls_after_cache_build)


if __name__ == "__main__":
    unittest.main()
