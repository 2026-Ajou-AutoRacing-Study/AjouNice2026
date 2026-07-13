#!/usr/bin/env python3

import rospy
from copy import deepcopy
import math

from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit
from autoware_perception_msgs.msg import PredictedObjects

from mvp_utils import object_pose, path_cumulative_lengths, project_pose_to_path


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

    def closest_obstacle_distance(self, point_s):
        if not self.objects or len(self.objects.objects) == 0 or self.path is None:
            return float('inf')
        dmin = float('inf')
        for obj in self.objects.objects:
            proj = project_pose_to_path(self.path, object_pose(obj))
            if proj is None or abs(proj.signed_lateral_distance) > self.max_lateral_distance:
                continue
            delta = proj.longitudinal_distance - point_s
            if delta < 0.0:
                continue
            dmin = min(dmin, delta)
        return dmin

    def slowdown_factor(self, d):
        if d >= self.slowdown_distance:
            return 1.0
        if d <= self.stop_distance:
            return 0.0
        return max(0.0, (d - self.stop_distance) / max(self.slowdown_distance - self.stop_distance, 1e-6))

    def publish_slowed_path(self):
        if not self.path:
            return
        out = deepcopy(self.path)
        distances = path_cumulative_lengths(self.path)
        for point, point_s in zip(out.points, distances):
            d = self.closest_obstacle_distance(point_s)

            factor = self.slowdown_factor(d)
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
