#!/usr/bin/env python3

import rospy
from copy import deepcopy
import math

from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit
from autoware_perception_msgs.msg import ObjectClassification, PredictedObjects

from mvp_utils import (
    object_pose,
    object_speed,
    feasible_stop_velocity,
    param_bool,
    path_cumulative_lengths,
    project_pose_to_path,
)


class DynamicObstacleStop:
    """Stops the vehicle for dynamic obstacles only.

    This module checks predicted objects and forces a stop if objects are within
    a dynamic stop distance. It ignores static obstacles beyond the stop range.
    """

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.planning_hz = float(rospy.get_param('~planning_hz', 10.0))
        self.default_velocity_mps = max(float(rospy.get_param('~default_velocity_mps', 5.0)), 0.0)
        self.use_velocity_limit = param_bool(rospy.get_param('~use_velocity_limit', True))
        self.dynamic_stop_distance = float(rospy.get_param('~dynamic_stop_distance', 5.0))
        self.warning_distance = float(rospy.get_param('~warning_distance', 10.0))
        self.min_velocity = float(rospy.get_param('~min_velocity_mps', 0.5))
        self.max_deceleration = float(rospy.get_param('~max_deceleration', 2.0))
        self.reaction_time = max(float(rospy.get_param('~reaction_time', 0.8)), 0.0)
        self.comfortable_deceleration = max(
            abs(float(rospy.get_param('~comfortable_deceleration', 3.0))),
            0.1,
        )
        self.object_size_margin = max(float(rospy.get_param('~object_size_margin', 0.0)), 0.0)
        self.dynamic_object_time_margin = max(
            float(rospy.get_param('~dynamic_object_time_margin', 1.0)),
            0.0,
        )
        self.slow_dynamic_stop_speed_threshold = max(
            float(rospy.get_param('~slow_dynamic_stop_speed_threshold_mps', 1.0)),
            0.0,
        )
        self.dynamic_stop_ttc_threshold = max(
            float(rospy.get_param('~dynamic_stop_ttc_threshold', 4.0)),
            0.0,
        )
        self.dynamic_stop_min_relative_speed = max(
            float(rospy.get_param('~dynamic_stop_min_relative_speed_mps', 1.0)),
            0.0,
        )
        self.dynamic_speed_threshold = float(rospy.get_param('~dynamic_speed_threshold_mps', 0.1))
        self.distance_buffer = float(rospy.get_param('~distance_buffer', 0.0))
        self.max_lateral_distance = float(rospy.get_param('~max_lateral_distance', 1.5))
        self.min_confidence = float(rospy.get_param('~min_confidence', 0.5))
        self.stop_unknown_objects = param_bool(rospy.get_param('~stop_unknown_objects', False))
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

        self.path = None
        self.objects = None
        self.velocity_limit = None
        self.odometry = None

        input_path_topic = rospy.get_param('~input_path_topic', '~input/path')
        objects_topic = rospy.get_param('~objects_topic', '~input/objects')
        odometry_topic = rospy.get_param('~odometry_topic', '~input/odometry')
        max_velocity_topic = rospy.get_param('~max_velocity_topic', '/planning/scenario_planning/max_velocity')
        output_path_topic = rospy.get_param('~output_path_topic', '~output/path')

        self.sub_path = rospy.Subscriber(input_path_topic, PathWithLaneId, self.cb_path)
        self.sub_objects = rospy.Subscriber(objects_topic, PredictedObjects, self.cb_objects)
        self.sub_odometry = rospy.Subscriber(odometry_topic, Odometry, self.cb_odometry)
        if max_velocity_topic:
            self.sub_velocity_limit = rospy.Subscriber(max_velocity_topic, VelocityLimit, self.cb_velocity_limit)

        self.pub_path = rospy.Publisher(output_path_topic, PathWithLaneId, queue_size=1)

        rospy.loginfo(
            'DynamicObstacleStop initialized (dynamic_stop_distance=%.2f, ttc_threshold=%.2f)',
            self.dynamic_stop_distance,
            self.dynamic_stop_ttc_threshold,
        )

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.publish_dynamic_stop_path()

    def cb_objects(self, msg: PredictedObjects):
        self.objects = msg
        self.publish_dynamic_stop_path()

    def cb_velocity_limit(self, msg: VelocityLimit):
        self.velocity_limit = msg
        self.publish_dynamic_stop_path()

    def cb_odometry(self, msg: Odometry):
        self.odometry = msg
        self.publish_dynamic_stop_path()

    def current_target_velocity(self):
        if self.use_velocity_limit and self.velocity_limit is not None:
            max_velocity = getattr(self.velocity_limit, 'max_velocity', 0.0)
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

    def speed_based_stop_distance(self, obj):
        ego_speed = self.ego_speed_for_stop_distance()
        reaction_distance = ego_speed * self.reaction_time
        braking_distance = (ego_speed * ego_speed) / (2.0 * self.comfortable_deceleration)
        dynamic_margin = max(ego_speed - object_speed(obj), 0.0) * self.dynamic_object_time_margin
        return max(
            0.0,
            self.dynamic_stop_distance
            + reaction_distance
            + braking_distance
            + self.object_size_margin
            + dynamic_margin,
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
        label = getattr(classification, 'label', None)
        probability = float(
            getattr(classification, 'probability', getattr(classification, 'confidence', 0.0))
        )
        if probability < self.min_confidence:
            return False

        if isinstance(label, str):
            label = label.lower()
            if label in self.stop_object_labels:
                return True
            return self.stop_unknown_objects and label == 'unknown'

        if isinstance(label, int):
            if label in self.stop_object_label_ids:
                return True
            return self.stop_unknown_objects and label == ObjectClassification.UNKNOWN

        return False

    def is_dynamic_object(self, obj):
        if not obj.classification:
            return False
        classification = obj.classification[0]
        probability = float(
            getattr(classification, 'probability', getattr(classification, 'confidence', 0.0))
        )
        if probability < self.min_confidence:
            return False
        speed = object_speed(obj)
        return speed > self.dynamic_speed_threshold

    def should_dynamic_stop_object(self, obj, proj):
        if not self.is_stop_object(obj):
            return False

        speed = object_speed(obj)
        if speed <= self.slow_dynamic_stop_speed_threshold:
            return True

        if self.relative_speed_to_object(obj) < self.dynamic_stop_min_relative_speed:
            return False

        return self.ttc_to_object(obj, proj) <= self.dynamic_stop_ttc_threshold

    def dynamic_object_projection(self, obj):
        if not self.path or not self.is_dynamic_object(obj):
            return None

        proj = project_pose_to_path(self.path, object_pose(obj))
        if proj is None:
            return None
        if proj.longitudinal_distance < 0.0:
            return None
        if abs(proj.signed_lateral_distance) > self.max_lateral_distance:
            return None
        return proj

    def publish_dynamic_stop_path(self):
        if not self.path:
            return

        out = deepcopy(self.path)
        distances = path_cumulative_lengths(out)
        stop_positions = []
        if self.objects:
            for obj in self.objects.objects:
                proj = self.dynamic_object_projection(obj)
                if proj is None or not self.should_dynamic_stop_object(obj, proj):
                    continue
                stop_distance = self.speed_based_stop_distance(obj)
                stop_positions.append(max(0.0, proj.longitudinal_distance - stop_distance))

        stop_line = min(stop_positions) if stop_positions else float('inf')

        for point, path_dist in zip(out.points, distances):
            if path_dist >= stop_line:
                target_velocity = 0.0
            else:
                remaining = stop_line - path_dist
                target_velocity = feasible_stop_velocity(remaining, self.max_deceleration)
                if path_dist >= stop_line - self.warning_distance:
                    target_velocity = min(target_velocity, self.min_velocity)

            if hasattr(point.point, 'longitudinal_velocity_mps'):
                try:
                    target_velocity = min(target_velocity, float(point.point.longitudinal_velocity_mps))
                except Exception:
                    pass

            if self.velocity_limit is not None and hasattr(self.velocity_limit, 'max_velocity'):
                try:
                    target_velocity = min(target_velocity, float(self.velocity_limit.max_velocity))
                except Exception:
                    pass

            point.point.longitudinal_velocity_mps = target_velocity

        self.pub_path.publish(out)

    def run(self):
        rate = rospy.Rate(self.planning_hz)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_stop')
    node = DynamicObstacleStop()
    node.run()
