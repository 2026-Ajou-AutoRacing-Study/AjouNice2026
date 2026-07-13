#!/usr/bin/env python3

import rospy
import math
from copy import deepcopy

from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit
from autoware_perception_msgs.msg import PredictedObjects

from mvp_utils import object_pose, object_speed, path_cumulative_lengths, project_pose_to_path


class ObstacleVelocityLimiter:
    """Obstacle velocity limiter based on predicted time-to-collision.

    This module adjusts path point velocities to maintain a minimum time-to-collision
    against nearby obstacles, while respecting an optional global velocity limit.
    """

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        self.min_ttc = max(float(rospy.get_param("~min_ttc", 1.0)), 1.0e-3)
        self.distance_buffer = float(rospy.get_param("~distance_buffer", 0.0))
        self.min_adjusted_velocity = float(rospy.get_param("~min_adjusted_velocity_mps", 2.5))
        self.max_deceleration = max(
            abs(float(rospy.get_param("~max_deceleration", 2.0))), 1.0e-3
        )
        self.max_lateral_distance = float(rospy.get_param("~max_lateral_distance", 1.5))

        self.path = None
        self.objects = None
        self.velocity_limit = None
        self.odometry = None
        self.ego_speed = 0.0

        input_path_topic = rospy.get_param('~input_path_topic', '~input/path')
        objects_topic = rospy.get_param('~objects_topic', '~input/objects')
        odometry_topic = rospy.get_param('~odometry_topic', '~input/odometry')
        max_velocity_topic = rospy.get_param('~max_velocity_topic', '/planning/scenario_planning/max_velocity')
        output_path_topic = rospy.get_param('~output_path_topic', '~output/path')

        self.sub_path = rospy.Subscriber(input_path_topic, PathWithLaneId, self.cb_path)
        self.sub_objects = rospy.Subscriber(objects_topic, PredictedObjects, self.cb_objects)
        if max_velocity_topic:
            self.sub_velocity_limit = rospy.Subscriber(max_velocity_topic, VelocityLimit, self.cb_velocity_limit)
        self.sub_odometry = rospy.Subscriber(odometry_topic, Odometry, self.cb_odometry)

        self.pub_path = rospy.Publisher(output_path_topic, PathWithLaneId, queue_size=1)

        rospy.loginfo(
            "ObstacleVelocityLimiter initialized (min_ttc=%.2f, distance_buffer=%.2f)",
            self.min_ttc,
            self.distance_buffer,
        )

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.apply_limits_and_publish()

    def cb_objects(self, msg: PredictedObjects):
        self.objects = msg
        self.apply_limits_and_publish()

    def cb_velocity_limit(self, msg: VelocityLimit):
        self.velocity_limit = msg
        self.apply_limits_and_publish()

    def cb_odometry(self, msg: Odometry):
        self.odometry = msg
        self.ego_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        self.apply_limits_and_publish()

    def obstacle_info(self, point_s):
        if not self.objects or len(self.objects.objects) == 0 or self.path is None:
            return float('inf'), float('inf')

        min_distance = float('inf')
        min_ttc = float('inf')
        for obj in self.objects.objects:
            proj = project_pose_to_path(self.path, object_pose(obj))
            if proj is None or abs(proj.signed_lateral_distance) > self.max_lateral_distance:
                continue

            distance = proj.longitudinal_distance - self.distance_buffer - point_s
            if distance <= 0.0:
                return 0.0, 0.0

            ego_speed = max(self.ego_speed, 0.1)
            obj_speed = object_speed(obj)
            relative_speed = max(ego_speed - obj_speed, 0.1)
            ttc = distance / relative_speed
            min_distance = min(min_distance, distance)
            min_ttc = min(min_ttc, ttc)

        return min_distance, min_ttc

    def apply_limits_and_publish(self):
        if not self.path:
            return

        out = deepcopy(self.path)
        distances = path_cumulative_lengths(self.path)
        for item, point_s in zip(out.points, distances):
            original_velocity = float(getattr(item.point, "longitudinal_velocity_mps", 0.0))
            distance, ttc = self.obstacle_info(point_s)

            if distance <= 0.0 or ttc <= 0.0:
                target_velocity = 0.0
            elif ttc <= self.min_ttc:
                safe_velocity = max(self.min_adjusted_velocity, distance / self.min_ttc)
                if original_velocity > 0.0:
                    target_velocity = min(original_velocity, safe_velocity)
                else:
                    target_velocity = safe_velocity
            else:
                target_velocity = original_velocity if original_velocity > 0.0 else self.min_adjusted_velocity

            if self.velocity_limit is not None and hasattr(self.velocity_limit, 'max_velocity'):
                try:
                    target_velocity = min(target_velocity, float(self.velocity_limit.max_velocity))
                except Exception:
                    pass

            item.point.longitudinal_velocity_mps = target_velocity

        self.enforce_max_deceleration(out, distances)
        self.pub_path.publish(out)

    def enforce_max_deceleration(self, path, distances):
        """Make the velocity profile physically reachable before a lower-speed point.

        The TTC rule can request an abrupt speed decrease at a point close to an
        obstacle.  Propagating the deceleration constraint backward gives upstream
        points enough distance to reach that request without exceeding the configured
        maximum deceleration.
        """
        for index in range(len(path.points) - 2, -1, -1):
            next_velocity = max(
                float(path.points[index + 1].point.longitudinal_velocity_mps), 0.0
            )
            segment_length = max(distances[index + 1] - distances[index], 0.0)
            reachable_velocity = math.sqrt(
                next_velocity * next_velocity
                + 2.0 * self.max_deceleration * segment_length
            )
            path.points[index].point.longitudinal_velocity_mps = min(
                float(path.points[index].point.longitudinal_velocity_mps),
                reachable_velocity,
            )

    def run(self):
        rate = rospy.Rate(self.planning_hz)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('obstacle_velocity_limiter')
    node = ObstacleVelocityLimiter()
    node.run()
