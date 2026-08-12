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
    interpolate_values_at_stations,
    project_motion_state,
    resample_profile_inputs,
    select_initial_state,
    smooth_velocity_profile,
    speed_dependent_curve_decel_distance,
)


class VelocitySmootherNode:
    def __init__(self):
        self.planning_hz = max(float(rospy.get_param("~planning_hz", 10.0)), 0.1)
        self.velocity_scale = max(
            float(rospy.get_param("~kinematic_velocity_scale_to_mps", 1.0)),
            0.0,
        )
        self.replan_velocity_deviation = max(
            float(rospy.get_param("~replan_velocity_deviation_mps", 0.5)),
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
            limit_max_deceleration_mps2=max(
                float(
                    rospy.get_param(
                        "~backward/limit_max_deceleration_mps2",
                        2.5,
                    )
                ),
                0.01,
            ),
            limit_max_deceleration_jerk_mps3=max(
                float(
                    rospy.get_param(
                        "~backward/limit_max_jerk_mps3",
                        1.5,
                    )
                ),
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
        self.resample_interval = max(
            float(rospy.get_param("~resample/interval_m", 0.1)),
            0.01,
        )
        self.stop_velocity_warn_threshold = max(
            float(
                rospy.get_param(
                    "~debug/stop_velocity_warn_threshold_mps",
                    0.3,
                )
            ),
            0.0,
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
        # Temporary vehicle-speed calibration for curve look-ahead distance:
        # 0 km/h -> 4 m, 20 km/h -> 6 m, 60 km/h -> 15 m,
        # and 120 km/h or faster -> 25 m.
        self.dynamic_curve_min_distance = 4.0
        self.dynamic_curve_low_speed = 20.0 / 3.6
        self.dynamic_curve_low_distance = 6.0
        self.dynamic_curve_high_speed = 60.0 / 3.6
        self.dynamic_curve_high_distance = 15.0
        self.dynamic_curve_maximum_speed = 120.0 / 3.6
        self.dynamic_curve_maximum_distance = 25.0
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
        self.curve_cap_release_hold_cycles = 5
        self.curve_cap_hold_match_distance = 2.0

        self.trajectory = None
        self.odometry = None
        self.previous_output = None
        self.curve_cap_cache = []
        self.curve_cap_hold_remaining = 0

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

    def stabilize_curve_caps(self, positions, raw_caps, curve_caps, curve_cap_count):
        """Delay only curve-cap release, while applying new lower caps immediately."""
        if curve_cap_count > 0:
            self.curve_cap_cache = [
                (position[0], position[1], float(cap))
                for position, raw, cap in zip(positions, raw_caps, curve_caps)
                if float(cap) + 1.0e-6 < float(raw)
            ]
            self.curve_cap_hold_remaining = self.curve_cap_release_hold_cycles
            return tuple(curve_caps), curve_cap_count, False

        if self.curve_cap_hold_remaining <= 0 or not self.curve_cap_cache:
            return tuple(curve_caps), curve_cap_count, False

        output = list(curve_caps)
        maximum_distance_squared = self.curve_cap_hold_match_distance ** 2
        for index, position in enumerate(positions):
            nearest = min(
                self.curve_cap_cache,
                key=lambda cached: (cached[0] - position[0]) ** 2
                + (cached[1] - position[1]) ** 2,
            )
            distance_squared = (nearest[0] - position[0]) ** 2 + (
                nearest[1] - position[1]
            ) ** 2
            if distance_squared <= maximum_distance_squared:
                output[index] = min(float(output[index]), nearest[2])
        self.curve_cap_hold_remaining -= 1
        held_count = sum(
            1
            for raw, cap in zip(raw_caps, output)
            if float(cap) + 1.0e-6 < float(raw)
        )
        return tuple(output), held_count, held_count > 0

    def previous_planned_state(self):
        if self.previous_output is None:
            return None, None, None
        current_position = self.current_position()
        if current_position is None:
            return None, None, None
        positions = self.positions_from_trajectory(self.previous_output)
        velocities = self.velocities_from_trajectory(self.previous_output)
        accelerations = [
            float(point.acceleration_mps2) for point in self.previous_output.points
        ]
        state = project_motion_state(
            positions,
            velocities,
            accelerations,
            current_position,
        )
        if state is None:
            return None, None, None
        if state.distance_m > self.previous_state_max_distance:
            return None, None, state.distance_m
        return (
            state.velocity_mps,
            state.acceleration_mps2,
            state.distance_m,
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
        curve_hold_applied = False
        curve_decel_distance = self.decel_distance_before_curve
        if current_velocity is not None:
            curve_decel_distance = speed_dependent_curve_decel_distance(
                current_velocity,
                self.dynamic_curve_min_distance,
                self.dynamic_curve_low_speed,
                self.dynamic_curve_low_distance,
                self.dynamic_curve_high_speed,
                self.dynamic_curve_high_distance,
                self.dynamic_curve_maximum_speed,
                self.dynamic_curve_maximum_distance,
            )
        if self.enable_lateral_acceleration_limit:
            velocity_caps, curve_cap_count = apply_lateral_acceleration_limits(
                positions,
                raw_velocities,
                self.max_lateral_accel,
                self.min_curve_velocity,
                curve_decel_distance,
                self.decel_distance_after_curve,
                self.curvature_window,
                self.min_curve_radius,
                self.parameters.stop_velocity_threshold_mps,
                self.parameters.max_deceleration_mps2,
                self.parameters.max_deceleration_jerk_mps3,
                initial_state.velocity_mps,
            )
            velocity_caps, curve_cap_count, curve_hold_applied = self.stabilize_curve_caps(
                positions,
                raw_velocities,
                velocity_caps,
                curve_cap_count,
            )
        resampled = resample_profile_inputs(
            positions,
            velocity_caps,
            self.resample_interval,
        )
        dense_profile = smooth_velocity_profile(
            resampled.positions,
            resampled.velocity_caps,
            initial_velocity_mps=initial_state.velocity_mps,
            initial_acceleration_mps2=initial_state.acceleration_mps2,
            parameters=self.parameters,
        )
        profile_velocities = interpolate_values_at_stations(
            resampled.stations,
            dense_profile.velocities,
            resampled.input_stations,
        )
        profile_accelerations = interpolate_values_at_stations(
            resampled.stations,
            dense_profile.accelerations,
            resampled.input_stations,
        )
        velocities, engaged = apply_engage_velocity_floor(
            positions,
            profile_velocities,
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
            else profile_accelerations
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
        if (
            dense_profile.deceleration_mode == "infeasible"
            or not dense_profile.stop_feasible
            or dense_profile.stop_velocity_before_overwrite_mps
            > self.stop_velocity_warn_threshold
        ):
            rospy.logwarn_throttle(
                1.0,
                "velocity_smoother deceleration profile: mode=%s stop_feasible=%s "
                "velocity_before_zero=%.3f m/s",
                dense_profile.deceleration_mode,
                dense_profile.stop_feasible,
                dense_profile.stop_velocity_before_overwrite_mps,
            )
        self.publish_status(
            "velocity_smoother: points={}/{} targets={} curve_caps={} curve_hold={} curve_decel_m={:.2f} cap_clamps={} "
            "init={} prev_dist={} engaged={} ego_speed_mps={} decel_mode={} "
            "stop_feasible={} stop_pre_zero_mps={:.3f}".format(
                len(output.points),
                len(resampled.positions),
                len(dense_profile.deceleration_target_indices),
                curve_cap_count,
                curve_hold_applied,
                curve_decel_distance,
                dense_profile.velocity_cap_clamp_count,
                initial_state.source,
                "n/a"
                if previous_distance is None
                else "{:.2f}".format(previous_distance),
                engaged,
                "n/a" if current_velocity is None else "{:.2f}".format(current_velocity),
                dense_profile.deceleration_mode,
                dense_profile.stop_feasible,
                dense_profile.stop_velocity_before_overwrite_mps,
            )
        )


def main():
    rospy.init_node("velocity_smoother")
    VelocitySmootherNode()
    rospy.spin()


if __name__ == "__main__":
    main()
