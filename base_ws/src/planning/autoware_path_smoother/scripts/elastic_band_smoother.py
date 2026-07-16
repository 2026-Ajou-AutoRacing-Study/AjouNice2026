#!/usr/bin/env python3

import math
import time
from copy import deepcopy

import rospy
from autoware_internal_debug_msgs.msg import Float64Stamped, StringStamped
from autoware_planning_msgs.msg import Path, PathPoint, Trajectory, TrajectoryPoint
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from autoware_path_smoother.elastic_band import ElasticBand, ElasticBandParameters
from autoware_path_smoother.replan_checker import ReplanChecker
from autoware_path_smoother.trajectory_utils import (
    TrajectorySample,
    apply_input_velocity,
    extend_trajectory,
    set_zero_velocity_after_stop,
)


def yaw_from_quaternion(value):
    return math.atan2(
        2.0 * (value.w * value.z + value.x * value.y),
        1.0 - 2.0 * (value.y * value.y + value.z * value.z),
    )


def quaternion_from_tangent(first, second):
    dx = second.x - first.x
    dy = second.y - first.y
    dz = second.z - first.z
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, max(math.hypot(dx, dy), 1.0e-9))
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    return Quaternion(
        x=-math.sin(half_pitch) * math.sin(half_yaw),
        y=math.sin(half_pitch) * math.cos(half_yaw),
        z=math.cos(half_pitch) * math.sin(half_yaw),
        w=math.cos(half_pitch) * math.cos(half_yaw),
    )


def samples_from_path(message):
    return [
        TrajectorySample(
            x=float(item.pose.position.x),
            y=float(item.pose.position.y),
            z=float(item.pose.position.z),
            yaw=yaw_from_quaternion(item.pose.orientation),
            longitudinal_velocity_mps=float(item.longitudinal_velocity_mps),
            lateral_velocity_mps=float(item.lateral_velocity_mps),
            heading_rate_rps=float(item.heading_rate_rps),
            is_final=bool(item.is_final),
        )
        for item in message.points
    ]


def assign_pose(target, points, index):
    point = points[index]
    target.position.x = point.x
    target.position.y = point.y
    target.position.z = point.z
    if len(points) == 1:
        target.orientation.z = math.sin(point.yaw * 0.5)
        target.orientation.w = math.cos(point.yaw * 0.5)
        return
    first = points[index - 1] if index > 0 else points[index]
    second = points[index + 1] if index + 1 < len(points) else points[index]
    target.orientation = quaternion_from_tangent(first, second)


def make_path(source, points):
    message = Path()
    message.header = deepcopy(source.header)
    message.left_bound = deepcopy(source.left_bound)
    message.right_bound = deepcopy(source.right_bound)
    for index, point in enumerate(points):
        item = PathPoint()
        assign_pose(item.pose, points, index)
        item.longitudinal_velocity_mps = point.longitudinal_velocity_mps
        item.lateral_velocity_mps = point.lateral_velocity_mps
        item.heading_rate_rps = point.heading_rate_rps
        item.is_final = True
        message.points.append(item)
    return message


def make_trajectory(source, points):
    message = Trajectory()
    message.header = deepcopy(source.header)
    for index, point in enumerate(points):
        item = TrajectoryPoint()
        assign_pose(item.pose, points, index)
        item.longitudinal_velocity_mps = point.longitudinal_velocity_mps
        item.lateral_velocity_mps = point.lateral_velocity_mps
        item.acceleration_mps2 = point.acceleration_mps2
        item.heading_rate_rps = point.heading_rate_rps
        item.front_wheel_angle_rad = point.front_wheel_angle_rad
        item.rear_wheel_angle_rad = point.rear_wheel_angle_rad
        message.points.append(item)
    return message


class ElasticBandSmootherNode:
    def __init__(self):
        self.enable_debug_info = bool(rospy.get_param("~enable_debug_info", False))
        parameters = ElasticBandParameters(
            num_points=int(rospy.get_param("~elastic_band/common/num_points", 100)),
            delta_arc_length=float(
                rospy.get_param("~elastic_band/common/delta_arc_length", 1.0)
            ),
            num_joint_points=int(
                rospy.get_param("~elastic_band/clearance/num_joint_points", 3)
            ),
            clearance_for_fix=float(
                rospy.get_param("~elastic_band/clearance/clearance_for_fix", 0.0)
            ),
            clearance_for_joint=float(
                rospy.get_param("~elastic_band/clearance/clearance_for_joint", 0.1)
            ),
            clearance_for_smooth=float(
                rospy.get_param("~elastic_band/clearance/clearance_for_smooth", 0.1)
            ),
            smooth_weight=float(
                rospy.get_param("~elastic_band/weight/smooth_weight", 1.0)
            ),
            lat_error_weight=float(
                rospy.get_param("~elastic_band/weight/lat_error_weight", 0.001)
            ),
            max_iteration=int(rospy.get_param("~elastic_band/qp/max_iteration", 10000)),
            eps_abs=float(rospy.get_param("~elastic_band/qp/eps_abs", 1.0e-7)),
            eps_rel=float(rospy.get_param("~elastic_band/qp/eps_rel", 1.0e-7)),
            enable_warm_start=bool(
                rospy.get_param("~elastic_band/option/enable_warm_start", True)
            ),
            enable_optimization_validation=bool(
                rospy.get_param(
                    "~elastic_band/option/enable_optimization_validation", False
                )
            ),
            max_validation_error=float(
                rospy.get_param("~elastic_band/validation/max_error", 3.0)
            ),
            output_backward_traj_length=float(
                rospy.get_param("~common/output_backward_traj_length", 5.0)
            ),
        )
        self.output_interval = float(
            rospy.get_param("~common/output_delta_arc_length", 0.5)
        )
        self.smoother = ElasticBand(parameters)
        self.replan_checker = ReplanChecker(
            enabled=bool(rospy.get_param("~replan/enable", True)),
            max_path_shape_around_ego_lat_dist=float(
                rospy.get_param("~replan/max_path_shape_around_ego_lat_dist", 2.0)
            ),
            max_path_shape_forward_lon_dist=float(
                rospy.get_param("~replan/max_path_shape_forward_lon_dist", 100.0)
            ),
            max_path_shape_forward_lat_dist=float(
                rospy.get_param("~replan/max_path_shape_forward_lat_dist", 0.2)
            ),
            max_ego_moving_dist=float(
                rospy.get_param("~replan/max_ego_moving_dist", 5.0)
            ),
            max_goal_moving_dist=float(
                rospy.get_param("~replan/max_goal_moving_dist", 15.0)
            ),
            max_delta_time_sec=float(
                rospy.get_param("~replan/max_delta_time_sec", 0.0)
            ),
        )
        self.odometry = None
        self.previous_optimized = None

        self.path_pub = rospy.Publisher("~output/path", Path, queue_size=1)
        self.trajectory_pub = rospy.Publisher("~output/traj", Trajectory, queue_size=1)
        self.debug_eb_pub = rospy.Publisher("~debug/eb_traj", Trajectory, queue_size=1)
        self.debug_fixed_pub = rospy.Publisher("~debug/eb_fixed_traj", Trajectory, queue_size=1)
        self.debug_extended_pub = rospy.Publisher(
            "~debug/extended_traj", Trajectory, queue_size=1
        )
        self.time_string_pub = rospy.Publisher(
            "~debug/calculation_time", StringStamped, queue_size=1
        )
        self.time_float_pub = rospy.Publisher(
            "~debug/processing_time_ms", Float64Stamped, queue_size=1
        )
        rospy.Subscriber("~input/odometry", Odometry, self.on_odometry, queue_size=1)
        rospy.Subscriber("~input/path", Path, self.on_path, queue_size=1)

    def on_odometry(self, message):
        self.odometry = message

    @staticmethod
    def is_forward(points):
        if len(points) < 2:
            return True
        yaw = points[0].yaw
        direction_x = points[1].x - points[0].x
        direction_y = points[1].y - points[0].y
        return math.cos(yaw) * direction_x + math.sin(yaw) * direction_y >= 0.0

    def publish_debug_time(self, milliseconds, stamp):
        string_message = StringStamped()
        string_message.stamp = stamp
        string_message.data = "onPath:= {:.3f} [ms]".format(milliseconds)
        self.time_string_pub.publish(string_message)
        float_message = Float64Stamped()
        float_message.stamp = stamp
        float_message.data = milliseconds
        self.time_float_pub.publish(float_message)

    def on_path(self, message):
        if self.odometry is None:
            rospy.loginfo_throttle(5.0, "path_smoother: waiting for ego pose and twist")
            return
        if len(message.points) < 2:
            rospy.loginfo_throttle(5.0, "path_smoother: path has fewer than two points")
            return
        if not message.left_bound or not message.right_bound:
            rospy.loginfo_throttle(5.0, "path_smoother: path bounds are empty")
            return

        started = time.perf_counter()
        input_points = samples_from_path(message)
        if not self.is_forward(input_points):
            rospy.logwarn_throttle(5.0, "path_smoother: backward path is not supported")
            output_path = make_path(message, input_points)
            self.path_pub.publish(output_path)
            self.trajectory_pub.publish(make_trajectory(message, input_points))
            return

        ego = self.odometry.pose.pose.position
        ego_value = (float(ego.x), float(ego.y))
        now = rospy.Time.now().to_sec()
        reset = self.replan_checker.reset_required(input_points, ego_value)
        if reset:
            self.smoother.reset()
            self.previous_optimized = None
        replan = (
            self.previous_optimized is None
            or self.replan_checker.replan_required(input_points, now)
        )
        optimized = (
            self.smoother.smooth(input_points, ego.x, ego.y)
            if replan
            else list(self.previous_optimized)
        )
        self.previous_optimized = list(optimized)
        self.replan_checker.update(input_points, ego_value, replan, now)

        self.debug_eb_pub.publish(make_trajectory(message, optimized))
        self.debug_fixed_pub.publish(
            make_trajectory(message, self.smoother.fixed_points)
        )
        velocity_applied = apply_input_velocity(optimized, input_points)
        full_points = extend_trajectory(input_points, velocity_applied, self.output_interval)
        full_points = set_zero_velocity_after_stop(full_points)

        output_path = make_path(message, full_points)
        output_trajectory = make_trajectory(message, full_points)
        self.path_pub.publish(output_path)
        self.trajectory_pub.publish(output_trajectory)
        self.debug_extended_pub.publish(output_trajectory)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.publish_debug_time(elapsed_ms, rospy.Time.now())
        if self.enable_debug_info:
            rospy.loginfo(
                "path_smoother: input=%d optimized=%d output=%d time=%.3f ms",
                len(input_points),
                len(optimized),
                len(full_points),
                elapsed_ms,
            )


def main():
    rospy.init_node("autoware_path_smoother")
    ElasticBandSmootherNode()
    rospy.spin()


if __name__ == "__main__":
    main()
