#!/usr/bin/env python3

import os
import sys
import unittest
from unittest.mock import patch

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


def moving_object(x, y, velocity_x, velocity_y):
    obj = predicted_object(x, y, ObjectClassification.CAR)
    obj.kinematics.initial_twist_with_covariance.twist.linear.x = float(velocity_x)
    obj.kinematics.initial_twist_with_covariance.twist.linear.y = float(velocity_y)
    obj.shape.dimensions.x = 4.5
    obj.shape.dimensions.y = 1.9
    return obj


def collision_planner():
    planner = MotionVelocityPlanner.__new__(MotionVelocityPlanner)
    planner.odometry = Odometry()
    planner.odometry.pose.pose.orientation.w = 1.0
    planner.use_velocity_limit = False
    planner.velocity_limit = None
    planner.default_velocity_mps = 5.0
    planner.delta_yaw_threshold = 1.0472
    planner.min_classification_confidence = 0.5
    planner.stop_unknown_objects = True
    planner.stop_object_labels = []
    planner.stop_object_label_ids = {ObjectClassification.CAR}
    planner.dynamic_speed_threshold = 0.1
    planner.dynamic_prediction_horizon = 8.0
    planner.dynamic_collision_path_resolution = 0.5
    planner.dynamic_collision_margin = 0.3
    planner.dynamic_collision_debug = False
    planner.autoware_stop_max_deceleration = 1.0
    planner.autoware_stop_max_deceleration_jerk = 1.0
    planner.max_lateral_distance = 1.5
    planner.vehicle_length = 4.635
    planner.vehicle_width = 1.892
    planner.vehicle_rear_overhang = 0.79
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

    def test_dynamic_prediction_detects_crossing_vehicle(self):
        planner = collision_planner()
        message = trajectory([5.0] * 51)
        collision = planner.dynamic_collision_station(
            moving_object(5.0, -6.0, 0.0, 6.0),
            message,
            path_cumulative_lengths(message),
        )
        self.assertIsNotNone(collision)
        self.assertLess(collision[0], 10.0)

    def test_dynamic_prediction_detects_oncoming_vehicle(self):
        planner = collision_planner()
        message = trajectory([5.0] * 51)
        collision = planner.dynamic_collision_station(
            moving_object(30.0, 0.0, -5.0, 0.0),
            message,
            path_cumulative_lengths(message),
        )
        self.assertIsNotNone(collision)
        self.assertLess(collision[0], 25.0)

    def test_dynamic_prediction_does_not_discard_ahead_swept_overlap(self):
        planner = collision_planner()
        message = trajectory([5.0] * 51)
        # The object is moving away in the same lane.  Its current swept area
        # already overlaps the ego trajectory, so the planner must keep the
        # earliest overlap rather than wait until ego reaches the object.
        object_in_path = moving_object(8.0, 0.0, 3.0, 0.0)

        collision = planner.dynamic_collision_station(
            object_in_path,
            message,
            path_cumulative_lengths(message),
        )

        self.assertIsNotNone(collision)
        self.assertLess(collision[0], 6.0)

    def test_dynamic_prediction_ignores_parallel_adjacent_vehicle(self):
        planner = collision_planner()
        message = trajectory([5.0] * 51)
        collision = planner.dynamic_collision_station(
            moving_object(5.0, 6.0, 5.0, 0.0),
            message,
            path_cumulative_lengths(message),
        )
        self.assertIsNone(collision)

    def test_dynamic_stop_uses_the_run_up_to_collision_for_braking(self):
        planner = collision_planner()
        planner.dynamic_stop_distance = 5.0
        planner.object_size_margin = 0.0
        planner.reaction_time = 0.8
        planner.max_deceleration = 1.0
        planner.warning_distance = 6.0
        planner.min_stop_velocity = 0.5
        planner.objects = PredictedObjects()
        # The swept path crosses the ego path around x=45 m.  A 5 m safety
        # buffer plus 0.8 s reaction distance places the stop near x=33 m,
        # leaving the preceding 33 m as the braking run-up.
        planner.objects.objects.append(moving_object(50.0, -10.0, 0.0, 6.0))
        message = trajectory([8.0] * 101)
        with patch("motion_velocity_planner.rospy.loginfo_throttle"), patch(
            "motion_velocity_planner.rospy.logwarn_throttle"
        ):
            planner.apply_dynamic_stop(message, path_cumulative_lengths(message))
        first_stop = next(
            index
            for index, point in enumerate(message.points)
            if point.longitudinal_velocity_mps == 0.0
        )
        self.assertGreater(first_stop, 25)
        self.assertLess(first_stop, 40)

    def test_autoware_braking_distance_increases_with_speed(self):
        planner = collision_planner()

        slow = planner.autoware_braking_distance(2.0)
        city_speed = planner.autoware_braking_distance(20.0 / 3.6)

        self.assertGreater(slow, 0.0)
        self.assertGreater(city_speed, slow)


if __name__ == "__main__":
    unittest.main()
