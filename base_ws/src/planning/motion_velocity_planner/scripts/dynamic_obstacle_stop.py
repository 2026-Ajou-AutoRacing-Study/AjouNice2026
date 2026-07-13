#!/usr/bin/env python3

import rospy
from copy import deepcopy
import math

from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit
from autoware_perception_msgs.msg import PredictedObjects

from mvp_utils import object_pose, object_speed, feasible_stop_velocity, path_cumulative_lengths, project_pose_to_path


class DynamicObstacleStop:
    """Stops the vehicle for dynamic obstacles only.

    This module checks predicted objects and forces a stop if objects are within
    a dynamic stop distance. It ignores static obstacles beyond the stop range.
    """

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.planning_hz = float(rospy.get_param('~planning_hz', 10.0))
        self.dynamic_stop_distance = float(rospy.get_param('~dynamic_stop_distance', 5.0))
        self.warning_distance = float(rospy.get_param('~warning_distance', 10.0))
        self.min_velocity = float(rospy.get_param('~min_velocity_mps', 0.5))
        self.max_lateral_distance = float(rospy.get_param('~max_lateral_distance', 1.5))
        self.min_confidence = float(rospy.get_param('~min_confidence', 0.5))

        self.path = None
        self.objects = None
        self.velocity_limit = None

        input_path_topic = rospy.get_param('~input_path_topic', '~input/path')
        objects_topic = rospy.get_param('~objects_topic', '~input/objects')
        max_velocity_topic = rospy.get_param('~max_velocity_topic', '/planning/scenario_planning/max_velocity')
        output_path_topic = rospy.get_param('~output_path_topic', '~output/path')

        self.sub_path = rospy.Subscriber(input_path_topic, PathWithLaneId, self.cb_path)
        self.sub_objects = rospy.Subscriber(objects_topic, PredictedObjects, self.cb_objects)
        if max_velocity_topic:
            self.sub_velocity_limit = rospy.Subscriber(max_velocity_topic, VelocityLimit, self.cb_velocity_limit)

        self.pub_path = rospy.Publisher(output_path_topic, PathWithLaneId, queue_size=1)

        rospy.loginfo('DynamicObstacleStop initialized (dynamic_stop_distance=%.2f)', self.dynamic_stop_distance)

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.publish_dynamic_stop_path()

    def cb_objects(self, msg: PredictedObjects):
        self.objects = msg
        self.publish_dynamic_stop_path()

    def cb_velocity_limit(self, msg: VelocityLimit):
        self.velocity_limit = msg
        self.publish_dynamic_stop_path()

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
        return speed > 0.1

    def dynamic_object_projection(self, obj):
        if not self.path or not self.is_dynamic_object(obj):
            return None

        proj = project_pose_to_path(self.path, obj.kinematics.initial_pose_with_covariance.pose)
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
                if proj is not None:
                    stop_positions.append(max(0.0, proj.longitudinal_distance - self.dynamic_stop_distance))

        stop_line = min(stop_positions) if stop_positions else float('inf')
        deceleration = float(rospy.get_param('~max_deceleration', 2.0))

        for point, path_dist in zip(out.points, distances):
            if path_dist >= stop_line:
                target_velocity = 0.0
            else:
                remaining = stop_line - path_dist
                target_velocity = feasible_stop_velocity(remaining, deceleration)
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
