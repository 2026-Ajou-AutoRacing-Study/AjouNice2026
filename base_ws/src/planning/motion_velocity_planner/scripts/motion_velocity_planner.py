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
    feasible_stop_velocity,
    param_bool,
    path_cumulative_lengths,
    point_velocity,
    project_pose_to_path,
    set_point_velocity,
)


class CachedObjectProjection:
    __slots__ = (
        "obj",
        "projection",
        "ahead_of_ego",
        "speed",
        "hard_stop",
        "dynamic",
    )

    def __init__(self, obj, projection, ahead_of_ego, speed, hard_stop, dynamic):
        self.obj = obj
        self.projection = projection
        self.ahead_of_ego = ahead_of_ego
        self.speed = speed
        self.hard_stop = hard_stop
        self.dynamic = dynamic


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
            "comfortable_deceleration=%.2f, dynamic_stop_ttc_threshold=%.2f)",
            self.default_velocity_mps,
            self.use_velocity_limit,
            self.reaction_time,
            self.comfortable_deceleration,
            self.dynamic_stop_ttc_threshold,
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

    def build_object_projection_cache(self, trajectory, objects, odometry):
        if trajectory is None or objects is None or len(objects.objects) == 0:
            return []

        ego_projection = None
        if odometry is not None:
            ego_projection = project_pose_to_path(
                trajectory,
                odometry.pose.pose,
                yaw_threshold=self.delta_yaw_threshold,
            )
            if ego_projection is None:
                ego_projection = project_pose_to_path(trajectory, odometry.pose.pose)

        cache = []
        for obj in objects.objects:
            projection = project_pose_to_path(trajectory, object_pose(obj))
            if projection is None:
                continue
            cache.append(
                CachedObjectProjection(
                    obj=obj,
                    projection=projection,
                    ahead_of_ego=(
                        ego_projection is None
                        or projection.longitudinal_distance
                        >= ego_projection.longitudinal_distance
                    ),
                    speed=object_speed(obj),
                    hard_stop=self.is_hard_stop_distance_object(obj),
                    dynamic=self.is_dynamic_object(obj),
                )
            )
        return cache

    def closest_obstacle_distance(
        self, point_s, hard_stop_only=False, object_projection_cache=None
    ):
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        if not object_projection_cache:
            return float('inf')

        dmin = float('inf')
        for cached in object_projection_cache:
            if hard_stop_only and not cached.hard_stop:
                continue

            proj = cached.projection
            if proj is None or proj.distance > self.max_lateral_distance:
                continue
            delta = proj.longitudinal_distance - point_s
            if delta < 0.0:
                continue
            dmin = min(dmin, delta)
        return dmin

    def obstacle_info(self, point_s, object_projection_cache=None):
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        if not object_projection_cache:
            return float('inf'), float('inf')

        min_distance = float('inf')
        min_ttc = float('inf')
        for cached in object_projection_cache:
            proj = cached.projection
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
            relative_speed = max(ego_speed - cached.speed, 0.1)
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

    def apply_stop(self, out, distances, object_projection_cache=None):
        stop_lines = []
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        for cached in object_projection_cache:
            if not cached.hard_stop:
                continue

            proj = cached.projection
            if (
                proj.longitudinal_distance < 0.0
                or proj.distance > self.max_lateral_distance
                or not cached.ahead_of_ego
            ):
                continue
            stop_distance = self.speed_based_stop_distance(self.stop_distance, cached.obj)
            stop_lines.append(max(0.0, proj.longitudinal_distance - stop_distance))

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

    def apply_dynamic_stop(self, out, distances, object_projection_cache=None):
        stop_lines = []
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        for cached in object_projection_cache:
            proj = cached.projection
            if (
                not cached.dynamic
                or proj.longitudinal_distance < 0.0
                or proj.distance > self.max_lateral_distance
                or not cached.ahead_of_ego
                or not self.should_dynamic_stop_object(cached.obj, proj)
            ):
                continue

            stop_distance = self.speed_based_stop_distance(
                self.dynamic_stop_distance,
                cached.obj,
                dynamic=True,
            )
            stop_lines.append(max(0.0, proj.longitudinal_distance - stop_distance))

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

    def apply_slowdown(self, out, distances, object_projection_cache=None):
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        stop_slow_distance = self.speed_based_stop_distance(self.stop_slow_distance)
        slowdown_distance = max(self.slowdown_distance, stop_slow_distance + 1.0)
        for point, station in zip(out.points, distances):
            d = self.closest_obstacle_distance(
                station,
                hard_stop_only=True,
                object_projection_cache=object_projection_cache,
            )
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

    def apply_cruise(self, out, distances, object_projection_cache=None):
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        for point, station in zip(out.points, distances):
            base_vel = self.cruise_speed
            current_velocity = point_velocity(point)
            if current_velocity > 0.0:
                base_vel = min(base_vel, current_velocity)

            d = self.closest_obstacle_distance(
                station,
                hard_stop_only=True,
                object_projection_cache=object_projection_cache,
            )
            if d <= 0.0:
                target_velocity = 0.0
            elif d <= self.safe_distance:
                t = d / max(self.safe_distance, 1e-6)
                target_velocity = max(self.min_slow_velocity, base_vel * t)
            else:
                target_velocity = base_vel

            set_point_velocity(point, min(point_velocity(point), target_velocity))

    def apply_ttc_limit(self, out, distances, object_projection_cache=None):
        if object_projection_cache is None:
            object_projection_cache = self.build_object_projection_cache(
                self.trajectory, self.objects, self.odometry
            )
        for point, station in zip(out.points, distances):
            current_velocity = point_velocity(point)
            distance, ttc = self.obstacle_info(
                station, object_projection_cache=object_projection_cache
            )
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

        trajectory = self.trajectory
        objects = self.objects
        odometry = self.odometry
        out = deepcopy(trajectory)
        self.apply_base_velocity(out)
        distances = path_cumulative_lengths(out)
        object_projection_cache = self.build_object_projection_cache(
            trajectory, objects, odometry
        )
        self.apply_stop(out, distances, object_projection_cache)
        self.apply_dynamic_stop(out, distances, object_projection_cache)
        self.apply_slowdown(out, distances, object_projection_cache)
        self.apply_cruise(out, distances, object_projection_cache)
        self.apply_ttc_limit(out, distances, object_projection_cache)
        if odometry is not None:
            out = crop_trajectory_around_ego(
                out,
                odometry.pose.pose,
                forward_length=self.extract_ahead_dist,
                backward_length=self.extract_behind_dist,
                yaw_threshold=self.delta_yaw_threshold,
            )
        if self.trajectory is not trajectory:
            rospy.logwarn_throttle(
                1.0,
                "motion_velocity_planner: discarding stale result because a newer trajectory arrived",
            )
            return
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
