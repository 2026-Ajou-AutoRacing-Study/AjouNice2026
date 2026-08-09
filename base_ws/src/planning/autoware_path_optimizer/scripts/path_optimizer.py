#!/usr/bin/env python3

import math
import time
from copy import deepcopy
from dataclasses import replace

import rospy
from autoware_internal_debug_msgs.msg import Float64Stamped, StringStamped
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path, Trajectory, TrajectoryPoint
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from autoware_path_optimizer.mpt_optimizer import MPTParameters, ModelPredictiveTrajectory
from autoware_path_optimizer.replan_checker import ReplanChecker
from autoware_path_optimizer.static_collision_checker import (
    first_static_collision,
    static_obstacles_from_message,
)
from autoware_path_optimizer.trajectory_utils import (
    TrajectorySample,
    apply_input_velocity,
    cumulative_lengths,
    join_with_remaining_path,
    resample,
    project_point,
    set_zero_velocity_after,
    set_zero_velocity_after_first_stop,
)


def yaw_from_quaternion(value):
    return math.atan2(
        2.0 * (value.w * value.z + value.x * value.y),
        1.0 - 2.0 * (value.y * value.y + value.z * value.z),
    )


def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))


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


def boundary_from_message(values):
    return [(float(point.x), float(point.y), float(point.z)) for point in values]


def make_trajectory(source, points):
    message = Trajectory()
    message.header = deepcopy(source.header)
    for point in points:
        item = TrajectoryPoint()
        item.pose.position.x = point.x
        item.pose.position.y = point.y
        item.pose.position.z = point.z
        item.pose.orientation = quaternion_from_yaw(point.yaw)
        item.longitudinal_velocity_mps = point.longitudinal_velocity_mps
        item.lateral_velocity_mps = point.lateral_velocity_mps
        item.acceleration_mps2 = point.acceleration_mps2
        item.heading_rate_rps = point.heading_rate_rps
        item.front_wheel_angle_rad = point.front_wheel_angle_rad
        item.rear_wheel_angle_rad = point.rear_wheel_angle_rad
        message.points.append(item)
    return message


class PathOptimizerNode:
    def __init__(self):
        self.enable_debug_info = bool(rospy.get_param("~debug/enable_debug_info", False))
        self.enable_skip_optimization = bool(
            rospy.get_param("~option/enable_skip_optimization", False)
        )
        self.enable_reset_previous = bool(
            rospy.get_param("~option/enable_reset_prev_optimization", False)
        )
        self.enable_outside_stop = bool(
            rospy.get_param("~option/enable_outside_drivable_area_stop", True)
        )
        self.output_interval = float(
            rospy.get_param("~common/output_delta_arc_length", 0.5)
        )
        self.stop_margin = max(
            float(
                rospy.get_param(
                    "~common/vehicle_stop_margin_outside_drivable_area", 1.0
                )
            ),
            0.0,
        )
        self.vehicle_width = float(rospy.get_param("~vehicle/width", 1.896))
        self.base_link_to_front = float(
            rospy.get_param("~vehicle/base_link_to_front", 3.845)
        )
        self.base_link_to_rear = float(
            rospy.get_param("~vehicle/base_link_to_rear", 0.790)
        )
        self.enable_static_collision_check = bool(
            rospy.get_param("~safety/enable_static_object_collision_check", True)
        )
        self.static_object_speed_threshold = max(
            float(rospy.get_param("~safety/static_object_speed_threshold", 1.0)),
            0.0,
        )
        self.static_object_margin = max(
            float(rospy.get_param("~safety/static_object_margin", 0.3)),
            0.0,
        )
        self.collision_sampling_interval = max(
            float(rospy.get_param("~safety/collision_sampling_interval", 0.25)),
            0.05,
        )
        self.collision_forward_distance = max(
            float(rospy.get_param("~safety/collision_forward_distance", 60.0)),
            1.0,
        )
        self.static_object_stop_margin = max(
            float(rospy.get_param("~safety/static_object_stop_margin", 1.0)),
            0.0,
        )
        self.allow_initial_margin_overlap_escape = bool(
            rospy.get_param(
                "~safety/allow_initial_margin_overlap_escape",
                True,
            )
        )
        self.maximum_margin_overlap_escape_distance = max(
            float(
                rospy.get_param(
                    "~safety/maximum_margin_overlap_escape_distance",
                    1.0,
                )
            ),
            0.0,
        )
        self.kinematic_validation_forward_distance = max(
            float(
                rospy.get_param(
                    "~mpt/validation/forward_distance", 50.0
                )
            ),
            1.0,
        )
        parameters = MPTParameters(
            num_points=int(rospy.get_param("~mpt/common/num_points", 100)),
            delta_arc_length=float(
                rospy.get_param("~mpt/common/delta_arc_length", 1.0)
            ),
            output_backward_traj_length=float(
                rospy.get_param("~common/output_backward_traj_length", 5.0)
            ),
            vehicle_width=self.vehicle_width,
            wheel_base=float(rospy.get_param("~vehicle/wheel_base", 2.79)),
            max_steer_angle=float(
                rospy.get_param("~vehicle/max_steer_angle", 0.61)
            ),
            soft_clearance_from_road=float(
                rospy.get_param("~mpt/clearance/soft_clearance_from_road", 0.1)
            ),
            min_drivable_width=float(
                rospy.get_param("~mpt/avoidance/min_drivable_width", 0.2)
            ),
            num_fixed_points=int(rospy.get_param("~mpt/fixed_points", 3)),
            lat_error_weight=float(
                rospy.get_param("~mpt/weight/lat_error_weight", 1.0)
            ),
            yaw_error_weight=float(
                rospy.get_param("~mpt/weight/yaw_error_weight", 2.0)
            ),
            steer_input_weight=float(
                rospy.get_param("~mpt/weight/steer_input_weight", 8.0)
            ),
            steer_rate_weight=float(
                rospy.get_param("~mpt/weight/steer_rate_weight", 30.0)
            ),
            terminal_lat_error_weight=float(
                rospy.get_param("~mpt/weight/terminal_lat_error_weight", 100.0)
            ),
            goal_lat_error_weight=float(
                rospy.get_param("~mpt/weight/goal_lat_error_weight", 1000.0)
            ),
            temporal_consistency_weight=float(
                rospy.get_param("~mpt/weight/temporal_consistency_weight", 20.0)
            ),
            temporal_consistency_decay_start=float(
                rospy.get_param(
                    "~mpt/weight/temporal_consistency_decay_start", 2.0
                )
            ),
            temporal_consistency_decay_end=float(
                rospy.get_param(
                    "~mpt/weight/temporal_consistency_decay_end", 10.0
                )
            ),
            max_iteration=int(rospy.get_param("~mpt/qp/max_iteration", 2000)),
            tolerance=float(rospy.get_param("~mpt/qp/tolerance", 1.0e-8)),
            enable_warm_start=bool(
                rospy.get_param("~mpt/option/enable_warm_start", True)
            ),
            enable_optimization_validation=bool(
                rospy.get_param(
                    "~mpt/option/enable_optimization_validation", True
                )
            ),
            max_validation_lat_error=float(
                rospy.get_param("~mpt/validation/max_lat_error", 5.0)
            ),
            max_curvature_rate=float(
                rospy.get_param("~mpt/validation/max_curvature_rate", 0.15)
            ),
        )
        self.parameters = parameters
        self.optimizer = ModelPredictiveTrajectory(parameters)
        self.replan_checker = ReplanChecker(
            max_path_shape_around_ego_lat_dist=float(
                rospy.get_param("~replan/max_path_shape_around_ego_lat_dist", 2.0)
            ),
            max_path_shape_forward_lon_dist=float(
                rospy.get_param("~replan/max_path_shape_forward_lon_dist", 100.0)
            ),
            max_path_shape_forward_lat_dist=float(
                rospy.get_param("~replan/max_path_shape_forward_lat_dist", 0.1)
            ),
            max_path_shape_forward_reset_lat_dist=float(
                rospy.get_param(
                    "~replan/max_path_shape_forward_reset_lat_dist", 0.3
                )
            ),
            max_ego_moving_dist=float(
                rospy.get_param("~replan/max_ego_moving_dist", 5.0)
            ),
            max_goal_moving_dist=float(
                rospy.get_param("~replan/max_goal_moving_dist", 15.0)
            ),
            max_delta_time_sec=float(
                rospy.get_param("~replan/max_delta_time_sec", 1.0)
            ),
        )
        self.odometry = None
        self.objects = None
        self.previous_geometry = None
        self.previous_output = None

        self.output_pub = rospy.Publisher("~output/path", Trajectory, queue_size=1)
        self.reference_pub = rospy.Publisher(
            "~debug/mpt_ref_traj", Trajectory, queue_size=1
        )
        self.fixed_pub = rospy.Publisher(
            "~debug/mpt_fixed_traj", Trajectory, queue_size=1
        )
        self.optimized_pub = rospy.Publisher(
            "~debug/mpt_traj", Trajectory, queue_size=1
        )
        self.extended_pub = rospy.Publisher(
            "~debug/extended_traj", Trajectory, queue_size=1
        )
        self.status_pub = rospy.Publisher("~debug/status", String, queue_size=1)
        self.time_string_pub = rospy.Publisher(
            "~debug/calculation_time", StringStamped, queue_size=1
        )
        self.time_float_pub = rospy.Publisher(
            "~debug/processing_time_ms", Float64Stamped, queue_size=1
        )
        rospy.Subscriber("~input/odometry", Odometry, self.on_odometry, queue_size=1)
        rospy.Subscriber(
            "~input/objects", PredictedObjects, self.on_objects, queue_size=1
        )
        rospy.Subscriber("~input/path", Path, self.on_path, queue_size=1)

    def on_odometry(self, message):
        self.odometry = message

    def on_objects(self, message):
        self.objects = message

    def first_static_collision(self, points, ego_position):
        if not self.enable_static_collision_check or self.objects is None:
            return None
        obstacles = static_obstacles_from_message(
            self.objects,
            self.static_object_speed_threshold,
            self.static_object_margin,
        )
        collision = first_static_collision(
            points,
            obstacles,
            ego_position,
            vehicle_width=self.vehicle_width,
            base_link_to_front=self.base_link_to_front,
            base_link_to_rear=self.base_link_to_rear,
            sampling_interval=self.collision_sampling_interval,
            backward_distance=0.0,
            forward_distance=self.collision_forward_distance,
            ignore_initial_overlap=self.allow_initial_margin_overlap_escape,
            maximum_initial_overlap_distance=(
                self.maximum_margin_overlap_escape_distance
            ),
        )
        if collision is not None:
            return collision

        # An initial overlap may only be ignored when it comes exclusively
        # from the configured safety margin.  A true footprint collision still
        # requires the normal fallback/stop response.
        physical_obstacles = static_obstacles_from_message(
            self.objects,
            self.static_object_speed_threshold,
            0.0,
        )
        return first_static_collision(
            points,
            physical_obstacles,
            ego_position,
            vehicle_width=self.vehicle_width,
            base_link_to_front=self.base_link_to_front,
            base_link_to_rear=self.base_link_to_rear,
            sampling_interval=self.collision_sampling_interval,
            backward_distance=0.0,
            forward_distance=self.collision_forward_distance,
        )

    @staticmethod
    def first_stop_station(points):
        stations = cumulative_lengths(points)
        for index, point in enumerate(points):
            if abs(point.longitudinal_velocity_mps) < 1.0e-4:
                return float(stations[index])
        return None

    def stop_before_static_collision(self, points, collision, ego_position):
        if not points:
            return []
        stations = cumulative_lengths(points)
        _, ego_station, _, _ = project_point(
            points, ego_position[0], ego_position[1]
        )
        stop_station = max(
            float(ego_station),
            float(collision.station) - self.static_object_stop_margin,
        )
        if stop_station <= float(ego_station) + 1.0e-6:
            stop_index = next(
                (
                    index
                    for index, station in enumerate(stations)
                    if float(station) >= float(ego_station) - 1.0e-6
                ),
                len(points) - 1,
            )
        else:
            stop_index = 0
            for index, station in enumerate(stations):
                if float(station) > stop_station + 1.0e-6:
                    break
                stop_index = index
        return set_zero_velocity_after(points, stop_index)

    def publish_time(self, milliseconds):
        stamp = rospy.Time.now()
        string_message = StringStamped(stamp=stamp, data="onPath:= {:.3f} [ms]".format(milliseconds))
        float_message = Float64Stamped(stamp=stamp, data=milliseconds)
        self.time_string_pub.publish(string_message)
        self.time_float_pub.publish(float_message)

    def collision_free(self, result):
        if not result.success:
            return False
        return all(
            lower - 1.0e-5 <= offset <= upper + 1.0e-5
            for offset, lower, upper in zip(
                result.offsets, result.lower_bounds, result.upper_bounds
            )
        )

    def stop_before_outside(
        self, points, left_bound, right_bound, ego_position=None
    ):
        if not self.enable_outside_stop or not points:
            return list(points)
        lower, upper = self.optimizer.lateral_bounds(points, left_bound, right_bound)
        stations = cumulative_lengths(points)
        ego_station = 0.0
        if ego_position is not None:
            _, ego_station, _, _ = project_point(
                points, ego_position[0], ego_position[1]
            )
        outside_index = next(
            (
                index
                for index, (station, low, high) in enumerate(
                    zip(stations, lower, upper)
                )
                if station >= ego_station - 1.0e-3
                if low > 0.0 or high < 0.0 or low > high
            ),
            None,
        )
        if outside_index is None:
            return list(points)
        stop_station = max(
            float(stations[outside_index]) - self.stop_margin,
            ego_station,
        )
        stop_index = int(max(0, min(len(points) - 1, (stations <= stop_station).sum() - 1)))
        return set_zero_velocity_after(points, stop_index)

    def fallback(self, source, left_bound, right_bound, ego_position=None):
        # A stale previous output eventually places ego behind the reusable
        # trajectory window. Keep fallback rolling with the current input path.
        geometry = resample(source, self.output_interval, preserve_stop=True)
        geometry = self.stop_before_outside(
            geometry, left_bound, right_bound, ego_position
        )
        return set_zero_velocity_after_first_stop(geometry)

    def on_path(self, message):
        if self.odometry is None:
            rospy.loginfo_throttle(5.0, "path_optimizer: waiting for ego odometry")
            return
        if len(message.points) < 2:
            rospy.loginfo_throttle(5.0, "path_optimizer: input path is too short")
            return
        if len(message.left_bound) < 2 or len(message.right_bound) < 2:
            rospy.logwarn_throttle(5.0, "path_optimizer: drivable bounds are empty")
            return
        started = time.perf_counter()
        source = samples_from_path(message)
        left_bound = boundary_from_message(message.left_bound)
        right_bound = boundary_from_message(message.right_bound)
        ego = self.odometry.pose.pose.position
        ego_position = (float(ego.x), float(ego.y))
        now = rospy.Time.now().to_sec()
        reset = self.enable_reset_previous or self.replan_checker.reset_required(
            source, ego_position
        )
        if reset:
            self.optimizer.reset()
            self.previous_geometry = None
        should_replan = (
            self.previous_geometry is None
            or self.replan_checker.replan_required(source, now, ego_position)
        )
        status = "reused previous trajectory"
        result = None
        if self.enable_skip_optimization:
            optimized = resample(source, self.parameters.delta_arc_length, False)
            status = "optimization skipped"
        elif should_replan:
            result = self.optimizer.optimize(
                source, ego_position[0], ego_position[1], left_bound, right_bound
            )
            if self.collision_free(result):
                optimized = result.points
                self.previous_geometry = [replace(point) for point in optimized]
                status = result.status
            else:
                output = self.fallback(
                    source, left_bound, right_bound, ego_position
                )
                status = result.status if result is not None else "optimization failure"
                status = "fallback: {}".format(status)
                self.publish(message, output, result, status, started)
                self.replan_checker.update(source, ego_position, True, now)
                return
        else:
            optimized = [replace(point) for point in self.previous_geometry]

        optimized = apply_input_velocity(optimized, source)
        output = join_with_remaining_path(source, optimized, self.output_interval)
        output_stations = cumulative_lengths(output)
        validation_length = (
            self.parameters.output_backward_traj_length
            + self.kinematic_validation_forward_distance
        )
        validation_count = int((output_stations <= validation_length).sum())
        validation_output = output[:max(validation_count, 3)]
        if (
            not self.enable_skip_optimization
            and self.parameters.enable_optimization_validation
            and not self.optimizer.kinematically_feasible(validation_output)
        ):
            output = self.fallback(source, left_bound, right_bound, ego_position)
            status = "fallback: output kinematic validation failure"

        collision = self.first_static_collision(output, ego_position)
        if collision is not None:
            fallback = self.fallback(
                source, left_bound, right_bound, ego_position
            )
            fallback_collision = self.first_static_collision(
                fallback, ego_position
            )
            object_id = collision.object_id[:8] or "unknown"
            if fallback_collision is None:
                output = fallback
                status = (
                    "fallback: optimized trajectory collides with static object "
                    + object_id
                )
                # The upstream path is safe, so the previous MPT geometry is
                # the source of the collision. Do not reuse it next cycle.
                self.optimizer.reset()
                self.previous_geometry = None
            else:
                first_stop = self.first_stop_station(fallback)
                if (
                    first_stop is not None
                    and first_stop <= fallback_collision.station + 1.0e-3
                ):
                    output = fallback
                    status = (
                        "fallback: upstream stop protects static object "
                        + object_id
                    )
                else:
                    output = self.stop_before_static_collision(
                        fallback,
                        fallback_collision,
                        ego_position,
                    )
                    status = "fallback: inserted stop for static object " + object_id
                    self.optimizer.reset()
                    self.previous_geometry = None
        output = set_zero_velocity_after_first_stop(output)
        output = [
            replace(
                point,
                heading_rate_rps=0.0,
                front_wheel_angle_rad=0.0,
                rear_wheel_angle_rad=0.0,
            )
            for point in output
        ]
        self.previous_output = [replace(point) for point in output]
        self.publish(message, output, result, status, started)
        self.replan_checker.update(source, ego_position, should_replan, now)

    def publish(self, source_message, output, result, status, started):
        output_message = make_trajectory(source_message, output)
        self.output_pub.publish(output_message)
        self.extended_pub.publish(output_message)
        self.status_pub.publish(String(data=status))
        if result is not None:
            self.reference_pub.publish(make_trajectory(source_message, result.reference_points))
            self.fixed_pub.publish(make_trajectory(source_message, result.fixed_points))
            if result.points:
                self.optimized_pub.publish(make_trajectory(source_message, result.points))
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.publish_time(elapsed_ms)
        if self.enable_debug_info:
            rospy.loginfo(
                "path_optimizer: status=%s output=%d time=%.3f ms",
                status,
                len(output),
                elapsed_ms,
            )


def main():
    rospy.init_node("autoware_path_optimizer")
    PathOptimizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
