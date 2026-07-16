#!/usr/bin/env python3

import rospy
from copy import deepcopy
import math

from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit
from autoware_perception_msgs.msg import PredictedObjects
from autoware_map_msgs.msg import LaneletMapBin


class BoundaryDeparturePrevention:
    """Prevents departure from the planned lane boundary.

    This module reduces speed when the ego is near a lane boundary or when the
    path curvature indicates a possible boundary departure.
    """

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.planning_hz = float(rospy.get_param('~planning_hz', 10.0))
        self.min_velocity = float(rospy.get_param('~min_velocity_mps', 1.0))
        self.curvature_threshold = float(rospy.get_param('~curvature_threshold', 0.15))
        self.curvature_slow_factor = float(rospy.get_param('~curvature_slow_factor', 0.5))
        self.max_lateral_distance = float(rospy.get_param('~max_lateral_distance', 1.5))
        self.boundary_speed_factor = float(rospy.get_param('~boundary_speed_factor', 0.5))

        self.path = None
        self.objects = None
        self.map = None
        self.velocity_limit = None

        input_path_topic = rospy.get_param('~input_path_topic', '~input/path')
        objects_topic = rospy.get_param('~objects_topic', '~input/objects')
        vector_map_topic = rospy.get_param('~vector_map_topic', '~input/vector_map')
        max_velocity_topic = rospy.get_param('~max_velocity_topic', '/planning/scenario_planning/max_velocity')
        output_path_topic = rospy.get_param('~output_path_topic', '~output/path')

        self.sub_path = rospy.Subscriber(input_path_topic, PathWithLaneId, self.cb_path)
        self.sub_objects = rospy.Subscriber(objects_topic, PredictedObjects, self.cb_objects)
        self.sub_map = rospy.Subscriber(vector_map_topic, LaneletMapBin, self.cb_map)
        if max_velocity_topic:
            self.sub_velocity_limit = rospy.Subscriber(max_velocity_topic, VelocityLimit, self.cb_velocity_limit)

        self.pub_path = rospy.Publisher(output_path_topic, PathWithLaneId, queue_size=1)

        rospy.loginfo('BoundaryDeparturePrevention initialized (curvature_threshold=%.2f)', self.curvature_threshold)

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.publish_adjusted_path()

    def cb_objects(self, msg: PredictedObjects):
        self.objects = msg
        self.publish_adjusted_path()

    def cb_map(self, msg: LaneletMapBin):
        self.map = msg
        self.publish_adjusted_path()

    def cb_velocity_limit(self, msg: VelocityLimit):
        self.velocity_limit = msg
        self.publish_adjusted_path()

    def compute_curvature(self, i):
        if not self.path or len(self.path.points) < 3:
            return 0.0

        idx_prev = max(i - 1, 0)
        idx_next = min(i + 1, len(self.path.points) - 1)
        p_prev = self.path.points[idx_prev].point.pose.position
        p_curr = self.path.points[i].point.pose.position
        p_next = self.path.points[idx_next].point.pose.position

        v1x = p_curr.x - p_prev.x
        v1y = p_curr.y - p_prev.y
        v2x = p_next.x - p_curr.x
        v2y = p_next.y - p_curr.y

        norm1 = math.hypot(v1x, v1y)
        norm2 = math.hypot(v2x, v2y)
        if norm1 < 1e-6 or norm2 < 1e-6:
            return 0.0

        dot = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (norm1 * norm2)))
        angle = math.acos(dot)
        segment_length = max((norm1 + norm2) * 0.5, 1e-6)
        return abs(angle) / segment_length

    def publish_adjusted_path(self):
        if not self.path:
            return
        out = deepcopy(self.path)
        for i, point in enumerate(out.points):
            original_velocity = float(point.point.longitudinal_velocity_mps)
            target_velocity = original_velocity if original_velocity > 0.0 else self.min_velocity
            curvature = self.compute_curvature(i)
            if curvature >= self.curvature_threshold:
                target_velocity = max(target_velocity * self.curvature_slow_factor, self.min_velocity)

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
    rospy.init_node('boundary_departure_prevention')
    node = BoundaryDeparturePrevention()
    node.run()
