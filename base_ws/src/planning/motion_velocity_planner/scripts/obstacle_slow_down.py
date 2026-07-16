#!/usr/bin/env python3

import rospy
from copy import deepcopy
import math

from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit
from autoware_perception_msgs.msg import ObjectClassification, PredictedObjects

from mvp_utils import object_pose, object_speed, param_bool, path_cumulative_lengths, project_pose_to_path


class ObstacleSlowDown:
    """Gradually slows down ahead of obstacles.

    This module reduces the longitudinal velocity of path points when predicted
    objects are within a slowdown distance, but does not force a full stop.
    """

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.planning_hz = float(rospy.get_param('~planning_hz', 10.0))
        self.slowdown_distance = float(rospy.get_param('~slowdown_distance', 15.0))
        self.stop_distance = float(rospy.get_param('~stop_distance', 3.0))
        self.min_velocity = float(rospy.get_param('~min_velocity_mps', 1.0))
        self.cruise_velocity = float(rospy.get_param('~cruise_velocity_mps', 5.0))
        self.max_lateral_distance = float(rospy.get_param('~max_lateral_distance', 1.5))
        self.reaction_time = max(float(rospy.get_param('~reaction_time', 0.8)), 0.0)
        self.comfortable_deceleration = max(
            abs(float(rospy.get_param('~comfortable_deceleration', 3.0))),
            0.1,
        )
        self.object_size_margin = max(float(rospy.get_param('~object_size_margin', 0.0)), 0.0)
        self.slow_dynamic_stop_speed_threshold = max(
            float(rospy.get_param('~slow_dynamic_stop_speed_threshold_mps', 1.0)),
            0.0,
        )
        self.stop_unknown_objects = param_bool(rospy.get_param('~stop_unknown_objects', False))
        self.min_classification_confidence = float(rospy.get_param('~min_confidence', 0.5))
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

        rospy.loginfo('ObstacleSlowDown initialized (slowdown_distance=%.2f)', self.slowdown_distance)

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.publish_slowed_path()

    def cb_objects(self, msg: PredictedObjects):
        self.objects = msg
        self.publish_slowed_path()

    def cb_velocity_limit(self, msg: VelocityLimit):
        self.velocity_limit = msg
        self.publish_slowed_path()

    def cb_odometry(self, msg: Odometry):
        self.odometry = msg
        self.publish_slowed_path()

    def current_target_velocity(self):
        if self.velocity_limit is not None:
            max_velocity = getattr(self.velocity_limit, 'max_velocity', 0.0)
            if max_velocity > 0.0:
                return min(self.cruise_velocity, float(max_velocity))
        return self.cruise_velocity

    def ego_speed_for_stop_distance(self):
        odometry_speed = 0.0
        if self.odometry is not None:
            odometry_speed = math.hypot(
                self.odometry.twist.twist.linear.x,
                self.odometry.twist.twist.linear.y,
            )
        return max(odometry_speed, self.current_target_velocity(), 0.0)

    def speed_based_stop_distance(self):
        ego_speed = self.ego_speed_for_stop_distance()
        reaction_distance = ego_speed * self.reaction_time
        braking_distance = (ego_speed * ego_speed) / (2.0 * self.comfortable_deceleration)
        return max(0.0, self.stop_distance + reaction_distance + braking_distance + self.object_size_margin)

    def is_stop_object(self, obj):
        if not obj.classification:
            return False

        classification = obj.classification[0]
        label = getattr(classification, 'label', None)
        probability = float(
            getattr(classification, 'probability', getattr(classification, 'confidence', 0.0))
        )
        if probability < self.min_classification_confidence:
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

    def is_hard_stop_distance_object(self, obj):
        return self.is_stop_object(obj) and object_speed(obj) <= self.slow_dynamic_stop_speed_threshold

    def closest_obstacle_distance(self, point_s):
        if not self.objects or len(self.objects.objects) == 0 or self.path is None:
            return float('inf')
        dmin = float('inf')
        for obj in self.objects.objects:
            if not self.is_hard_stop_distance_object(obj):
                continue
            proj = project_pose_to_path(self.path, object_pose(obj))
            if proj is None or abs(proj.signed_lateral_distance) > self.max_lateral_distance:
                continue
            delta = proj.longitudinal_distance - point_s
            if delta < 0.0:
                continue
            dmin = min(dmin, delta)
        return dmin

    def slowdown_factor(self, d, stop_distance, slowdown_distance):
        if d >= slowdown_distance:
            return 1.0
        if d <= stop_distance:
            return 0.0
        return max(0.0, (d - stop_distance) / max(slowdown_distance - stop_distance, 1e-6))

    def publish_slowed_path(self):
        if not self.path:
            return
        out = deepcopy(self.path)
        distances = path_cumulative_lengths(self.path)
        stop_distance = self.speed_based_stop_distance()
        slowdown_distance = max(self.slowdown_distance, stop_distance + 1.0)
        for point, point_s in zip(out.points, distances):
            d = self.closest_obstacle_distance(point_s)

            factor = self.slowdown_factor(d, stop_distance, slowdown_distance)
            target_velocity = max(self.min_velocity, self.cruise_velocity * factor)
            if hasattr(point.point, 'longitudinal_velocity_mps'):
                try:
                    orig = float(point.point.longitudinal_velocity_mps)
                    if orig > 0.0:
                        target_velocity = min(target_velocity, orig)
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
    rospy.init_node('obstacle_slow_down')
    node = ObstacleSlowDown()
    node.run()
