#!/usr/bin/env python3

import os
import sys
import rospy
import math
from copy import deepcopy

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_perception_msgs.msg import PredictedObjects, ObjectClassification
from autoware_planning_msgs.msg import Trajectory

from mvp_utils import (
    crop_trajectory_around_ego,
    object_pose,
    object_speed,
    object_velocity_xy,
    oriented_box_corners,
    oriented_boxes_intersect,
    feasible_stop_velocity,
    param_bool,
    path_cumulative_lengths,
    point_velocity,
    project_pose_to_path,
    set_point_velocity,
    trajectory_point_at_distance,
    yaw_from_quaternion,
)


class MotionVelocityPlanner:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        self.default_velocity_mps = max(float(rospy.get_param("~default_velocity_mps", 5.0)), 0.0)
        self.use_velocity_limit = param_bool(rospy.get_param("~use_velocity_limit", True))
        self.extract_ahead_dist = max(
            float(rospy.get_param("~extract_ahead_dist", 200.0)), 1.0
        )
        self.extract_behind_dist = max(
            float(rospy.get_param("~extract_behind_dist", 0.0)), 0.0
        )
        self.delta_yaw_threshold = max(
            float(rospy.get_param("~delta_yaw_threshold", 1.0472)), 0.0
        )

        self.stop_distance = float(rospy.get_param("~stop_distance", 3.0))
        self.dynamic_stop_distance = float(rospy.get_param("~dynamic_stop_distance", 5.0))
        self.warning_distance = float(rospy.get_param("~warning_distance", 6.0))
        self.min_stop_velocity = float(rospy.get_param("~min_stop_velocity_mps", 0.5))
        self.max_deceleration = float(rospy.get_param("~max_deceleration", 2.0))
        self.reaction_time = max(float(rospy.get_param("~reaction_time", 0.8)), 0.0)
        self.comfortable_deceleration = max(
            abs(float(rospy.get_param("~comfortable_deceleration", 3.0))),
            0.1,
        )
        self.object_size_margin = max(float(rospy.get_param("~object_size_margin", 0.0)), 0.0)
        self.dynamic_object_time_margin = max(
            float(rospy.get_param("~dynamic_object_time_margin", 1.0)),
            0.0,
        )
        self.slow_dynamic_stop_speed_threshold = max(
            float(rospy.get_param("~slow_dynamic_stop_speed_threshold_mps", 1.0)),
            0.0,
        )
        self.dynamic_stop_ttc_threshold = max(
            float(rospy.get_param("~dynamic_stop_ttc_threshold", 4.0)),
            0.0,
        )
        self.dynamic_stop_min_relative_speed = max(
            float(rospy.get_param("~dynamic_stop_min_relative_speed_mps", 1.0)),
            0.0,
        )
        # Immediate-path collision check, modelled after Autoware's dynamic
        # obstacle stop module.  MORAI provides a current pose and velocity,
        # so use a constant-velocity, constant-heading prediction here.
        self.dynamic_prediction_horizon = max(
            float(rospy.get_param("~dynamic_prediction_horizon_sec", 8.0)), 0.1
        )
        self.dynamic_collision_path_resolution = max(
            float(rospy.get_param("~dynamic_collision_path_resolution_m", 0.5)), 0.05
        )
        self.dynamic_collision_margin = max(
            float(rospy.get_param("~dynamic_collision_margin_m", 0.3)), 0.0
        )
        self.dynamic_collision_debug = param_bool(
            rospy.get_param("~dynamic_collision_debug", False)
        )
        self.vehicle_length = max(float(rospy.get_param("~vehicle_length_m", 4.635)), 0.1)
        self.vehicle_width = max(float(rospy.get_param("~vehicle_width_m", 1.892)), 0.1)
        self.vehicle_rear_overhang = max(
            float(rospy.get_param("~vehicle_rear_overhang_m", 0.79)), 0.0
        )

        self.slowdown_distance = float(rospy.get_param("~slowdown_distance", 15.0))
        self.stop_slow_distance = float(rospy.get_param("~stop_distance", 3.0))
        self.min_slow_velocity = float(rospy.get_param("~min_slow_velocity_mps", 1.0))
        self.cruise_speed = float(rospy.get_param("~cruise_speed_mps", 5.0))
        self.safe_distance = float(rospy.get_param("~safe_distance", 10.0))

        self.min_ttc = float(rospy.get_param("~min_ttc", 1.0))
        self.distance_buffer = float(rospy.get_param("~distance_buffer", 0.0))
        self.min_adjusted_velocity = float(rospy.get_param("~min_adjusted_velocity_mps", 2.5))
        self.max_lateral_distance = float(rospy.get_param("~max_lateral_distance", 1.5))
        self.dynamic_speed_threshold = float(rospy.get_param("~dynamic_speed_threshold_mps", 0.1))

        self.stop_unknown_objects = param_bool(rospy.get_param("~stop_unknown_objects", False))
        self.min_classification_confidence = float(rospy.get_param("~min_confidence", 0.5))
        self.stop_object_labels = [
            label.strip().lower()
            for label in rospy.get_param(
                '~stop_object_labels',
                ['car', 'truck', 'bus', 'bicycle', 'motorcycle', 'pedestrian'],
            )
            if isinstance(label, str)
        ]
        self.stop_object_label_ids = set()
        for label in self.stop_object_labels:
            if hasattr(ObjectClassification, label.upper()):
                self.stop_object_label_ids.add(getattr(ObjectClassification, label.upper()))

        self.odometry = None
        self.trajectory = None
        self.velocity_limit = None
        self.objects = None

        input_trajectory_topic = rospy.get_param(
            '~input_trajectory_topic',
            '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory',
        )
        objects_topic = rospy.get_param(
            '~objects_topic', '/perception/object_recognition/objects'
        )
        odometry_topic = rospy.get_param(
            '~odometry_topic', '/localization/kinematic_state'
        )
        max_velocity_topic = rospy.get_param('~max_velocity_topic', '/planning/scenario_planning/max_velocity')
        output_trajectory_topic = rospy.get_param(
            '~output_trajectory_topic',
            '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory',
        )

        self.sub_odometry = rospy.Subscriber(
            odometry_topic, Odometry, self.cb_odometry, queue_size=1
        )
        self.sub_trajectory = rospy.Subscriber(
            input_trajectory_topic, Trajectory, self.cb_trajectory, queue_size=1
        )
        self.sub_objects = rospy.Subscriber(
            objects_topic, PredictedObjects, self.cb_objects, queue_size=1
        )
        if max_velocity_topic:
            self.sub_velocity_limit = rospy.Subscriber(
                max_velocity_topic,
                VelocityLimit,
                self.cb_velocity_limit,
                queue_size=1,
            )

        self.pub_trajectory = rospy.Publisher(
            output_trajectory_topic, Trajectory, queue_size=1
        )
        self.planning_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.planning_hz, 0.1)), self.on_planning_timer
        )

        rospy.loginfo(
            "MotionVelocityPlanner initialized "
            "(default_velocity=%.2f, use_velocity_limit=%s, reaction_time=%.2f, "
            "comfortable_deceleration=%.2f, dynamic_stop_ttc_threshold=%.2f, "
            "dynamic_prediction_horizon=%.2f)",
            self.default_velocity_mps,
            self.use_velocity_limit,
            self.reaction_time,
            self.comfortable_deceleration,
            self.dynamic_stop_ttc_threshold,
            self.dynamic_prediction_horizon,
        )

    def cb_odometry(self, msg):
        self.odometry = msg
        rospy.loginfo_throttle(5.0, "motion_velocity_planner: received odometry")

    def cb_trajectory(self, msg):
        self.trajectory = msg
        rospy.loginfo_throttle(
            5.0,
            "motion_velocity_planner: received trajectory with %d points",
            len(msg.points),
        )

    def cb_objects(self, msg):
        self.objects = msg
        rospy.loginfo_throttle(
            5.0,
            "motion_velocity_planner: received objects with %d entries",
            len(msg.objects),
        )

    def cb_velocity_limit(self, msg):
        self.velocity_limit = msg

    def on_planning_timer(self, _event):
        self.apply_velocity_profile()

    def current_target_velocity(self):
        if self.use_velocity_limit and self.velocity_limit is not None:
            max_velocity = getattr(self.velocity_limit, "max_velocity", 0.0)
            if max_velocity > 0.0:
                return min(self.default_velocity_mps, float(max_velocity))
        return self.default_velocity_mps

    def ego_speed_for_stop_distance(self):
        odometry_speed = 0.0
        if self.odometry is not None:
            odometry_speed = math.hypot(
                self.odometry.twist.twist.linear.x,
                self.odometry.twist.twist.linear.y,
            )
        return max(odometry_speed, self.current_target_velocity(), 0.0)

    def speed_based_stop_distance(self, base_distance, obj=None, dynamic=False):
        ego_speed = self.ego_speed_for_stop_distance()
        reaction_distance = ego_speed * self.reaction_time
        braking_distance = (ego_speed * ego_speed) / (2.0 * self.comfortable_deceleration)
        dynamic_margin = 0.0
        if dynamic and obj is not None:
            dynamic_margin = max(ego_speed - object_speed(obj), 0.0) * self.dynamic_object_time_margin
        return max(
            0.0,
            base_distance + reaction_distance + braking_distance + self.object_size_margin + dynamic_margin,
        )

    def relative_speed_to_object(self, obj):
        return max(self.ego_speed_for_stop_distance() - object_speed(obj), 0.0)

    def ttc_to_object(self, obj, proj, point_s=0.0):
        distance = proj.longitudinal_distance - self.distance_buffer - point_s
        if distance <= 0.0:
            return 0.0

        relative_speed = self.relative_speed_to_object(obj)
        if relative_speed <= 1.0e-6:
            return float('inf')
        return distance / relative_speed

    def is_stop_object(self, obj):
        if not obj.classification:
            return False

        classification = obj.classification[0]
        label_value = getattr(classification, 'label', None)
        confidence = float(getattr(classification, 'probability', getattr(classification, 'confidence', 0.0)))
        if confidence < self.min_classification_confidence:
            return False

        if isinstance(label_value, str):
            label = label_value.lower()
            if label in self.stop_object_labels:
                return True
            return self.stop_unknown_objects and label == 'unknown'

        if isinstance(label_value, int):
            if label_value in self.stop_object_label_ids:
                return True
            return self.stop_unknown_objects and label_value == ObjectClassification.UNKNOWN

        return False

    def is_dynamic_object(self, obj):
        if not obj.classification:
            return False

        confidence = float(
            getattr(obj.classification[0], 'probability', getattr(obj.classification[0], 'confidence', 0.0))
        )
        if confidence < self.min_classification_confidence:
            return False

        speed = object_speed(obj)
        return speed > self.dynamic_speed_threshold

    def is_hard_stop_distance_object(self, obj):
        return self.is_stop_object(obj) and object_speed(obj) <= self.slow_dynamic_stop_speed_threshold

    def is_projection_ahead_of_ego(self, proj):
        if self.odometry is None:
            return True

        ego_proj = project_pose_to_path(
            self.trajectory,
            self.odometry.pose.pose,
            yaw_threshold=self.delta_yaw_threshold,
        )
        if ego_proj is None:
            ego_proj = project_pose_to_path(self.trajectory, self.odometry.pose.pose)
        if ego_proj is None:
            return True
        return proj.longitudinal_distance >= ego_proj.longitudinal_distance

    def should_dynamic_stop_object(self, obj, proj):
        if not self.is_stop_object(obj):
            return False

        speed = object_speed(obj)
        if speed <= self.slow_dynamic_stop_speed_threshold:
            return True

        if self.relative_speed_to_object(obj) < self.dynamic_stop_min_relative_speed:
            return False

        return self.ttc_to_object(obj, proj) <= self.dynamic_stop_ttc_threshold

    def stop_object_projection(self, obj):
        if not self.trajectory or not self.is_stop_object(obj):
            return None

        proj = project_pose_to_path(self.trajectory, object_pose(obj))
        if proj is None:
            return None
        if proj.longitudinal_distance < 0.0:
            return None
        if proj.distance > self.max_lateral_distance:
            return None
        if not self.is_projection_ahead_of_ego(proj):
            return None
        return proj

    def dynamic_object_projection(self, obj):
        if not self.trajectory or not self.is_dynamic_object(obj):
            return None

        proj = project_pose_to_path(self.trajectory, object_pose(obj))
        if proj is None:
            return None
        if proj.longitudinal_distance < 0.0:
            return None
        if proj.distance > self.max_lateral_distance:
            return None
        if not self.is_projection_ahead_of_ego(proj):
            return None
        return proj

    def ego_station_on_path(self, path, distances):
        if self.odometry is None or not distances:
            return 0.0
        projection = project_pose_to_path(
            path,
            self.odometry.pose.pose,
            yaw_threshold=self.delta_yaw_threshold,
        )
        if projection is None:
            projection = project_pose_to_path(path, self.odometry.pose.pose)
        if projection is None:
            return 0.0
        return min(max(projection.longitudinal_distance, 0.0), distances[-1])

    def dynamic_collision_station(self, obj, path, distances):
        """Return Autoware-style earliest collision with an object's immediate path.

        The immediate path is the swept rectangle of a moving object during a
        fixed horizon.  Unlike the previous time-synchronised check, this does
        not assume the vehicle will exactly achieve the upstream target speed.
        It therefore remains safe when the velocity smoother or controller
        delays the ego arrival time.
        """
        if (
            not path.points
            or not distances
            or not self.is_stop_object(obj)
            or not self.is_dynamic_object(obj)
        ):
            return None

        ego_station = self.ego_station_on_path(path, distances)

        pose = object_pose(obj)
        velocity_x, velocity_y = object_velocity_xy(obj)
        object_speed_xy = math.hypot(velocity_x, velocity_y)
        object_yaw = (
            math.atan2(velocity_y, velocity_x)
            if object_speed_xy > self.dynamic_speed_threshold
            else yaw_from_quaternion(pose.orientation)
        )
        object_length = max(float(getattr(obj.shape.dimensions, "x", 0.0)), 0.1)
        object_width = max(float(getattr(obj.shape.dimensions, "y", 0.0)), 0.1)

        # The trajectory pose is at base_link/rear axle.  Move the ego box
        # centre forward so its footprint spans the actual vehicle body.
        ego_center_offset = (self.vehicle_length * 0.5) - self.vehicle_rear_overhang
        box_margin = self.dynamic_collision_margin * 2.0
        ego_length = self.vehicle_length + box_margin
        ego_width = self.vehicle_width + box_margin
        object_length += box_margin
        object_width += box_margin

        # Autoware Dynamic Obstacle Stop constructs one rectangle for the area
        # traversed by the object during the prediction horizon, then finds the
        # first ego trajectory footprint intersecting that rectangle.
        swept_length = object_length + object_speed_xy * self.dynamic_prediction_horizon
        swept_center_offset = object_speed_xy * self.dynamic_prediction_horizon * 0.5
        immediate_path_corners = oriented_box_corners(
            pose.position.x,
            pose.position.y,
            object_yaw,
            swept_length,
            object_width,
            swept_center_offset,
        )

        station = ego_station
        while station <= distances[-1] + 1.0e-6:
            ego_point = trajectory_point_at_distance(path, distances, station)
            if ego_point is None:
                return None
            ego_pose = ego_point.point.pose if hasattr(ego_point, "point") else ego_point.pose
            ego_corners = oriented_box_corners(
                ego_pose.position.x,
                ego_pose.position.y,
                yaw_from_quaternion(ego_pose.orientation),
                ego_length,
                ego_width,
                ego_center_offset,
            )
            # Ignore the portion behind an object moving away from ego.  This
            # is the same direction gate used by Autoware's module.
            object_to_ego_yaw = math.atan2(
                ego_pose.position.y - pose.position.y,
                ego_pose.position.x - pose.position.x,
            )
            heading_difference = abs(
                math.atan2(
                    math.sin(object_to_ego_yaw - object_yaw),
                    math.cos(object_to_ego_yaw - object_yaw),
                )
            )
            if (
                heading_difference <= 3.0 * math.pi / 4.0
                and oriented_boxes_intersect(ego_corners, immediate_path_corners)
            ):
                ego_speed = max(self.ego_speed_for_stop_distance(), 0.1)
                return station, max((station - ego_station) / ego_speed, 0.0)

            if station >= distances[-1] - 1.0e-6:
                break
            station = min(
                distances[-1], station + self.dynamic_collision_path_resolution
            )
        return None

    def closest_obstacle_distance(self, point_s, hard_stop_only=False):
        if not self.objects or len(self.objects.objects) == 0 or self.trajectory is None:
            return float('inf')

        dmin = float('inf')
        for obj in self.objects.objects:
            if hard_stop_only and not self.is_hard_stop_distance_object(obj):
                continue

            proj = project_pose_to_path(self.trajectory, object_pose(obj))
            if proj is None or proj.distance > self.max_lateral_distance:
                continue
            delta = proj.longitudinal_distance - point_s
            if delta < 0.0:
                continue
            dmin = min(dmin, delta)
        return dmin

    def obstacle_info(self, point_s):
        if not self.objects or len(self.objects.objects) == 0 or self.trajectory is None:
            return float('inf'), float('inf')

        min_distance = float('inf')
        min_ttc = float('inf')
        for obj in self.objects.objects:
            proj = project_pose_to_path(self.trajectory, object_pose(obj))
            if proj is None or proj.distance > self.max_lateral_distance:
                continue

            distance = proj.longitudinal_distance - self.distance_buffer - point_s
            if distance <= 0.0:
                continue

            ego_speed = 0.0
            if self.odometry is not None:
                ego_speed = math.hypot(
                    self.odometry.twist.twist.linear.x,
                    self.odometry.twist.twist.linear.y,
                )
            ego_speed = max(ego_speed, 0.1)
            obj_speed = object_speed(obj)
            relative_speed = max(ego_speed - obj_speed, 0.1)
            ttc = distance / relative_speed
            min_distance = min(min_distance, distance)
            min_ttc = min(min_ttc, ttc)

        return min_distance, min_ttc

    def apply_base_velocity(self, out):
        target_velocity = self.current_target_velocity()
        for point in out.points:
            current_velocity = point_velocity(point)
            if current_velocity > 0.0:
                set_point_velocity(point, min(current_velocity, target_velocity))

    def apply_stop(self, out, distances):
        if self.objects is None:
            return

        stop_lines = []
        debug_stop_lines = []
        for obj in self.objects.objects:
            if not self.is_hard_stop_distance_object(obj):
                continue

            proj = self.stop_object_projection(obj)
            if proj is not None:
                stop_distance = self.speed_based_stop_distance(self.stop_distance, obj)
                stop_line = max(0.0, proj.longitudinal_distance - stop_distance)
                stop_lines.append(stop_line)
                if self.dynamic_collision_debug:
                    debug_stop_lines.append("hard=s%.1f" % stop_line)

        if not stop_lines:
            return

        stop_line = min(stop_lines)
        if self.dynamic_collision_debug:
            rospy.logwarn_throttle(
                1.0,
                "motion_velocity_planner hard-stop candidates: %s; selected=s%.1f",
                ", ".join(debug_stop_lines),
                stop_line,
            )
        for point, station in zip(out.points, distances):
            if station >= stop_line:
                target_velocity = 0.0
            else:
                remaining = stop_line - station
                target_velocity = feasible_stop_velocity(remaining, self.max_deceleration)
                if station >= stop_line - self.warning_distance:
                    target_velocity = min(target_velocity, self.min_stop_velocity)
            set_point_velocity(point, min(point_velocity(point), target_velocity))

    def apply_dynamic_stop(self, out, distances):
        if self.objects is None:
            return

        ego_station = self.ego_station_on_path(out, distances)
        stop_lines = []
        debug_objects = []
        for obj in self.objects.objects:
            if self.dynamic_collision_debug and self.is_stop_object(obj) and self.is_dynamic_object(obj):
                pose = object_pose(obj).position
                velocity_x, velocity_y = object_velocity_xy(obj)
            else:
                pose = None
            collision = self.dynamic_collision_station(obj, out, distances)
            if collision is not None:
                collision_station, collision_time = collision
                # The stop point belongs immediately before the collision
                # point, not one full braking distance before it.  The
                # velocity profile then uses every metre from the current ego
                # position to that stop point for braking.  Subtracting the
                # braking distance here shortened the available braking run-up
                # and was the reason a detected collision could still result
                # in a late stop.
                ego_speed = self.ego_speed_for_stop_distance()
                stop_buffer = (
                    self.dynamic_stop_distance
                    + self.object_size_margin
                    + ego_speed * self.reaction_time
                )
                stop_line = max(ego_station, collision_station - stop_buffer)
                stop_lines.append(stop_line)
                available_distance = max(stop_line - ego_station, 0.0)
                required_braking_distance = (
                    (ego_speed * ego_speed) / (2.0 * max(self.max_deceleration, 0.1))
                )
                if available_distance < required_braking_distance:
                    rospy.logwarn_throttle(
                        1.0,
                        "motion_velocity_planner: dynamic stop is late "
                        "(available=%.1fm required=%.1fm); commanding maximum profile",
                        available_distance,
                        required_braking_distance,
                    )
                rospy.loginfo_throttle(
                    1.0,
                    "motion_velocity_planner: predicted dynamic collision in %.2fs at s=%.2f",
                    collision_time,
                    collision_station,
                )
                if pose is not None:
                    debug_objects.append(
                        "p=(%.1f,%.1f) v=(%.1f,%.1f) collision=s%.1f"
                        % (pose.x, pose.y, velocity_x, velocity_y, collision_station)
                    )
                continue

            if pose is not None:
                debug_objects.append(
                    "p=(%.1f,%.1f) v=(%.1f,%.1f) collision=none"
                    % (pose.x, pose.y, velocity_x, velocity_y)
                )

        if self.dynamic_collision_debug and debug_objects:
            rospy.logwarn_throttle(
                1.0,
                "motion_velocity_planner dynamic candidates: %s",
                " | ".join(debug_objects),
            )

        if not stop_lines:
            return

        stop_line = min(stop_lines)
        for point, station in zip(out.points, distances):
            if station >= stop_line:
                target_velocity = 0.0
            else:
                remaining = stop_line - station
                target_velocity = feasible_stop_velocity(remaining, self.max_deceleration)
                if station >= stop_line - self.warning_distance:
                    target_velocity = min(target_velocity, self.min_stop_velocity)
            set_point_velocity(point, min(point_velocity(point), target_velocity))

    def apply_slowdown(self, out, distances):
        stop_slow_distance = self.speed_based_stop_distance(self.stop_slow_distance)
        slowdown_distance = max(self.slowdown_distance, stop_slow_distance + 1.0)
        for point, station in zip(out.points, distances):
            d = self.closest_obstacle_distance(station, hard_stop_only=True)
            if d <= 0.0:
                target_velocity = 0.0
            elif d <= slowdown_distance:
                if d <= stop_slow_distance:
                    target_velocity = 0.0
                else:
                    factor = max(
                        0.0,
                        (d - stop_slow_distance)
                        / max(slowdown_distance - stop_slow_distance, 1e-6),
                    )
                    target_velocity = max(self.min_slow_velocity, self.cruise_speed * factor)
            else:
                target_velocity = point_velocity(point)
            set_point_velocity(point, min(point_velocity(point), target_velocity))

    def apply_cruise(self, out, distances):
        for point, station in zip(out.points, distances):
            base_vel = self.cruise_speed
            current_velocity = point_velocity(point)
            if current_velocity > 0.0:
                base_vel = min(base_vel, current_velocity)

            d = self.closest_obstacle_distance(station, hard_stop_only=True)
            if d <= 0.0:
                target_velocity = 0.0
            elif d <= self.safe_distance:
                t = d / max(self.safe_distance, 1e-6)
                target_velocity = max(self.min_slow_velocity, base_vel * t)
            else:
                target_velocity = base_vel

            set_point_velocity(point, min(point_velocity(point), target_velocity))

    def apply_ttc_limit(self, out, distances):
        for point, station in zip(out.points, distances):
            current_velocity = point_velocity(point)
            distance, ttc = self.obstacle_info(station)
            if distance <= 0.0 or ttc <= 0.0:
                target_velocity = 0.0
            elif ttc < self.min_ttc:
                safe_velocity = max(self.min_adjusted_velocity, distance / self.min_ttc)
                target_velocity = min(current_velocity if current_velocity > 0.0 else safe_velocity, safe_velocity)
            else:
                target_velocity = current_velocity if current_velocity > 0.0 else self.min_adjusted_velocity
            set_point_velocity(point, min(current_velocity, target_velocity))

    def apply_velocity_profile(self):
        if rospy.is_shutdown():
            return
        if self.trajectory is None:
            rospy.loginfo_throttle(
                5.0, "motion_velocity_planner: no trajectory yet, skipping profile"
            )
            return

        out = deepcopy(self.trajectory)
        self.apply_base_velocity(out)
        distances = path_cumulative_lengths(out)
        self.apply_stop(out, distances)
        self.apply_dynamic_stop(out, distances)
        self.apply_slowdown(out, distances)
        self.apply_cruise(out, distances)
        self.apply_ttc_limit(out, distances)
        if self.odometry is not None:
            out = crop_trajectory_around_ego(
                out,
                self.odometry.pose.pose,
                forward_length=self.extract_ahead_dist,
                backward_length=self.extract_behind_dist,
                yaw_threshold=self.delta_yaw_threshold,
            )
        rospy.loginfo_throttle(
            5.0,
            "motion_velocity_planner: publishing trajectory with %d points",
            len(out.points),
        )
        if not rospy.is_shutdown():
            try:
                self.pub_trajectory.publish(out)
            except rospy.ROSException:
                if not rospy.is_shutdown():
                    raise

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("motion_velocity_planner")
    planner = MotionVelocityPlanner()
    planner.run()
