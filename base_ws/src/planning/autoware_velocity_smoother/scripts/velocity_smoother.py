#!/usr/bin/env python3

"""ROS 1 node for the analytical jerk-constrained velocity smoother."""

from copy import deepcopy
import math

import rospy
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from autoware_velocity_smoother.analytical_jerk_constrained import (
    SmootherParameters,
    accelerations_from_velocity_profile,
    apply_lateral_acceleration_limits,
    apply_engage_velocity_floor,
    cumulative_lengths,
    nearest_index_2d,
    select_initial_state,
    smooth_velocity_profile,
)


class VelocitySmootherNode:
    def __init__(self):
        self.planning_hz = max(float(rospy.get_param("~planning_hz", 10.0)), 0.1)
        self.velocity_scale = max(
            float(rospy.get_param("~kinematic_velocity_scale_to_mps", 1.0)),
            0.0,
        )
        self.replan_velocity_deviation = max(
            float(rospy.get_param("~replan_velocity_deviation_mps", 5.53)),
            0.0,
        )
        self.previous_state_max_distance = max(
            float(rospy.get_param("~initial_state/max_previous_pose_distance_m", 5.0)),
            0.0,
        )
        self.engage_velocity = max(
            float(rospy.get_param("~engage_velocity_mps", 0.25)),
            0.0,
        )
        self.engage_distance = max(
            float(rospy.get_param("~engage_distance_m", 5.0)),
            0.0,
        )
        self.stop_distance_to_prohibit_engage = max(
            float(rospy.get_param("~stop_distance_to_prohibit_engage_m", 0.5)),
            0.0,
        )
        self.parameters = SmootherParameters(
            max_velocity_mps=max(float(rospy.get_param("~max_velocity_mps", 11.1)), 0.0),
            max_acceleration_mps2=max(
                float(rospy.get_param("~forward/max_acceleration_mps2", 1.0)),
                0.01,
            ),
            max_deceleration_mps2=max(
                float(rospy.get_param("~backward/max_deceleration_mps2", 2.5)),
                0.01,
            ),
            max_jerk_mps3=max(
                float(rospy.get_param("~forward/max_jerk_mps3", 1.0)),
                0.01,
            ),
            max_deceleration_jerk_mps3=max(
                float(rospy.get_param("~backward/max_jerk_mps3", 1.5)),
                0.01,
            ),
            min_velocity_for_time_calculation_mps=max(
                float(
                    rospy.get_param(
                        "~min_velocity_for_time_calculation_mps",
                        1.0,
                    )
                ),
                0.01,
            ),
            stop_velocity_threshold_mps=max(
                float(rospy.get_param("~stop_velocity_threshold_mps", 0.01)),
                0.0,
            ),
        )
        self.enable_lateral_acceleration_limit = bool(
            rospy.get_param("~curve/enable_lateral_acceleration_limit", True)
        )
        self.max_lateral_accel = max(
            float(rospy.get_param("~curve/max_lateral_accel_mps2", 1.5)),
            0.0,
        )
        self.min_curve_velocity = max(
            float(rospy.get_param("~curve/min_curve_velocity_mps", 1.0)),
            0.0,
        )
        self.decel_distance_before_curve = max(
            float(rospy.get_param("~curve/decel_distance_before_curve_m", 30.0)),
            0.0,
        )
        self.decel_distance_after_curve = max(
            float(rospy.get_param("~curve/decel_distance_after_curve_m", 4.0)),
            0.0,
        )
        self.curvature_window = max(
            int(rospy.get_param("~curve/curvature_window", 3)),
            1,
        )
        self.min_curve_radius = max(
            float(rospy.get_param("~curve/min_curve_radius_m", 1.0)),
            0.01,
        )

        self.trajectory = None
        self.odometry = None
        self.previous_output = None

        input_topic = rospy.get_param(
            "~input_trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/raw_trajectory",
        )
        output_topic = rospy.get_param(
            "~output_trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/trajectory",
        )
        odometry_topic = rospy.get_param(
            "~odometry_topic",
            "/localization/kinematic_state",
        )
        raw_debug_topic = rospy.get_param(
            "~debug_raw_trajectory_topic",
            "/planning/debug/velocity_smoother/raw_trajectory",
        )
        status_topic = rospy.get_param(
            "~debug_status_topic",
            "/planning/debug/velocity_smoother/status",
        )

        self.output_pub = rospy.Publisher(output_topic, Trajectory, queue_size=1)
        self.raw_debug_pub = rospy.Publisher(raw_debug_topic, Trajectory, queue_size=1)
        self.closest_velocity_pub = rospy.Publisher(
            "~closest_velocity", Float32, queue_size=1
        )
        self.closest_acceleration_pub = rospy.Publisher(
            "~closest_acceleration", Float32, queue_size=1
        )
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=1)
        self.trajectory_sub = rospy.Subscriber(
            input_topic,
            Trajectory,
            self.on_trajectory,
            queue_size=1,
        )
        self.odometry_sub = rospy.Subscriber(
            odometry_topic,
            Odometry,
            self.on_odometry,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.planning_hz),
            self.on_timer,
        )

        rospy.loginfo(
            "velocity_smoother initialized (hz=%.1f, max_acc=%.2f, max_decel=%.2f, "
            "max_decel_jerk=%.2f, curve_limit=%s, max_lat=%.2f)",
            self.planning_hz,
            self.parameters.max_acceleration_mps2,
            self.parameters.max_deceleration_mps2,
            self.parameters.max_deceleration_jerk_mps3,
            "on" if self.enable_lateral_acceleration_limit else "off",
            self.max_lateral_accel,
        )

    def on_trajectory(self, message):
        self.trajectory = message

    def on_odometry(self, message):
        self.odometry = message

    def current_velocity_mps(self):
        if self.odometry is None:
            return None
        linear = self.odometry.twist.twist.linear
        velocity = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        return velocity * self.velocity_scale

    @staticmethod
    def positions_from_trajectory(trajectory):
        return [
            (
                point.pose.position.x,
                point.pose.position.y,
                point.pose.position.z,
            )
            for point in trajectory.points
        ]

    @staticmethod
    def velocities_from_trajectory(trajectory):
        return [float(point.longitudinal_velocity_mps) for point in trajectory.points]

    def current_position(self):
        if self.odometry is None:
            return None
        position = self.odometry.pose.pose.position
        return position.x, position.y, position.z

    def previous_planned_state(self):
        if self.previous_output is None:
            return None, None, None
        current_position = self.current_position()
        if current_position is None:
            return None, None, None
        positions = self.positions_from_trajectory(self.previous_output)
        index, distance = nearest_index_2d(positions, current_position)
        if index < 0 or distance > self.previous_state_max_distance:
            return None, None, distance
        point = self.previous_output.points[index]
        return (
            float(point.longitudinal_velocity_mps),
            float(point.acceleration_mps2),
            distance,
        )

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def on_timer(self, _event):
        if self.trajectory is None:
            self.publish_status("velocity_smoother: waiting for input trajectory")
            return

        raw = deepcopy(self.trajectory)
        self.raw_debug_pub.publish(raw)
        if len(raw.points) < 2:
            self.output_pub.publish(raw)
            self.publish_status(
                "velocity_smoother: passthrough trajectory with {} point(s)".format(
                    len(raw.points)
                )
            )
            return

        current_velocity = self.current_velocity_mps()
        previous_velocity, previous_acceleration, previous_distance = (
            self.previous_planned_state()
        )
        initial_state = select_initial_state(
            current_velocity,
            previous_velocity,
            previous_acceleration,
            self.replan_velocity_deviation,
            0.0,
        )
        positions = self.positions_from_trajectory(raw)
        raw_velocities = self.velocities_from_trajectory(raw)
        velocity_caps = raw_velocities
        curve_cap_count = 0
        if self.enable_lateral_acceleration_limit:
            velocity_caps, curve_cap_count = apply_lateral_acceleration_limits(
                positions,
                raw_velocities,
                self.max_lateral_accel,
                self.min_curve_velocity,
                self.decel_distance_before_curve,
                self.decel_distance_after_curve,
                self.curvature_window,
                self.min_curve_radius,
                self.parameters.stop_velocity_threshold_mps,
                self.parameters.max_deceleration_mps2,
                self.parameters.max_deceleration_jerk_mps3,
                initial_state.velocity_mps,
            )
        profile = smooth_velocity_profile(
            positions,
            velocity_caps,
            initial_velocity_mps=initial_state.velocity_mps,
            initial_acceleration_mps2=initial_state.acceleration_mps2,
            parameters=self.parameters,
        )
        velocities, engaged = apply_engage_velocity_floor(
            positions,
            profile.velocities,
            velocity_caps,
            current_velocity,
            self.engage_velocity,
            self.engage_distance,
            self.stop_distance_to_prohibit_engage,
            self.parameters.stop_velocity_threshold_mps,
        )
        accelerations = (
            accelerations_from_velocity_profile(
                velocities,
                cumulative_lengths(positions),
            )
            if engaged
            else profile.accelerations
        )

        output = deepcopy(raw)
        output.header.stamp = rospy.Time.now()
        for point, velocity, acceleration in zip(
            output.points,
            velocities,
            accelerations,
        ):
            point.longitudinal_velocity_mps = velocity
            point.acceleration_mps2 = acceleration

        self.previous_output = deepcopy(output)
        self.output_pub.publish(output)
        self.closest_velocity_pub.publish(Float32(data=velocities[0]))
        self.closest_acceleration_pub.publish(Float32(data=accelerations[0]))
        self.publish_status(
            "velocity_smoother: points={} targets={} curve_caps={} cap_clamps={} "
            "init={} prev_dist={} engaged={} ego_speed_mps={}".format(
                len(output.points),
                len(profile.deceleration_target_indices),
                curve_cap_count,
                profile.velocity_cap_clamp_count,
                initial_state.source,
                "n/a"
                if previous_distance is None
                else "{:.2f}".format(previous_distance),
                engaged,
                "n/a" if current_velocity is None else "{:.2f}".format(current_velocity),
            )
        )


def main():
    rospy.init_node("velocity_smoother")
    VelocitySmootherNode()
    rospy.spin()


if __name__ == "__main__":
    main()
