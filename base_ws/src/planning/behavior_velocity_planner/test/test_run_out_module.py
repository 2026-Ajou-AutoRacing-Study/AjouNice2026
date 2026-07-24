import unittest

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
)

from behavior_velocity_planner_common import PlannerData
from behavior_velocity_planner_common.path_utils import to_path
from behavior_velocity_run_out_module import RunOutModuleManager, RunOutParameters


def path():
    message = PathWithLaneId()
    message.header.frame_id = "map"
    for x in range(0, 61):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.orientation.w = 1.0
        point.point.longitudinal_velocity_mps = 10.0
        point.lane_ids = [1]
        message.points.append(point)
    return message


def crossing_object(label=ObjectClassification.PEDESTRIAN):
    detected = PredictedObject()
    detected.object_id.uuid = list(range(16))
    detected.classification = [ObjectClassification(label=label, probability=1.0)]
    detected.shape.dimensions.y = 0.6
    detected.kinematics.initial_pose_with_covariance.pose.position.x = 20.0
    detected.kinematics.initial_pose_with_covariance.pose.position.y = -4.0
    detected.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
    detected.kinematics.initial_twist_with_covariance.twist.linear.y = 2.0
    prediction = PredictedPath()
    prediction.confidence = 1.0
    prediction.time_step = rospy.Duration.from_sec(0.5)
    for index in range(9):
        pose = Pose()
        pose.position.x = 20.0
        pose.position.y = -4.0 + index
        pose.orientation.w = 1.0
        prediction.path.append(pose)
    detected.kinematics.predicted_paths = [prediction]
    message = PredictedObjects()
    message.objects = [detected]
    return message


class RunOutTest(unittest.TestCase):
    def test_ego_arrival_time_uses_acceleration_from_stop(self):
        manager = RunOutModuleManager(
            RunOutParameters(expected_ego_acceleration=0.5)
        )
        self.assertAlmostEqual(manager._ego_arrival_time(25.0, 0.0, 10.0), 10.0)

    def test_collision_progresses_monitor_slowdown_stop(self):
        parameters = RunOutParameters(
            time_margin=0.6,
            slowdown_on_time_buffer=0.1,
            stop_on_time_buffer=0.3,
            base_link_to_front=0.0,
        )
        manager = RunOutModuleManager(parameters)
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 10.0
        message = path()
        decisions = []
        outputs = []
        for now in (10.0, 10.2, 10.4):
            output = to_path(message)
            result = manager.modify_path_velocity(
                message,
                output,
                PlannerData(
                    odometry=odometry,
                    dynamic_objects=crossing_object(),
                    current_time=now,
                ),
            )
            decisions.append(result.status)
            outputs.append(output)
        self.assertIn("MONITORING", decisions[0])
        self.assertIn("SLOW_DOWN", decisions[1])
        self.assertIn("STOP", decisions[2])
        self.assertTrue(any(point.longitudinal_velocity_mps == 0.0 for point in outputs[2].points))

    def test_non_vru_is_ignored(self):
        manager = RunOutModuleManager(RunOutParameters())
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 10.0
        message = path()
        result = manager.modify_path_velocity(
            message,
            to_path(message),
            PlannerData(odometry=odometry, dynamic_objects=crossing_object(ObjectClassification.CAR), current_time=10.0),
        )
        self.assertFalse(result.modified)

    def test_clear_scene_does_not_resample_public_path(self):
        manager = RunOutModuleManager(RunOutParameters(path_interpolation_ds=0.1))
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        message = path()
        output = to_path(message)
        original_size = len(output.points)

        result = manager.modify_path_velocity(
            message,
            output,
            PlannerData(
                odometry=odometry,
                dynamic_objects=PredictedObjects(),
                current_time=10.0,
            ),
        )

        self.assertFalse(result.modified)
        self.assertEqual(len(output.points), original_size)

    def test_collision_only_inserts_decision_points(self):
        parameters = RunOutParameters(
            time_margin=0.6,
            slowdown_on_time_buffer=0.0,
            stop_on_time_buffer=0.0,
            base_link_to_front=0.0,
            path_interpolation_ds=0.1,
        )
        manager = RunOutModuleManager(parameters)
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 10.0
        message = path()
        output = to_path(message)
        original_size = len(output.points)

        result = manager.modify_path_velocity(
            message,
            output,
            PlannerData(
                odometry=odometry,
                dynamic_objects=crossing_object(),
                current_time=10.0,
            ),
        )

        self.assertTrue(result.modified)
        self.assertLessEqual(len(output.points), original_size + 2)


if __name__ == "__main__":
    unittest.main()
