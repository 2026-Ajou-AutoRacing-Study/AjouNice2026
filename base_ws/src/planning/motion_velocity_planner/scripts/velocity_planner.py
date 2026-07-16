#!/usr/bin/env python3

import rospy
from copy import deepcopy

from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit


class VelocityPlanner:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        self.default_velocity = float(rospy.get_param("~default_velocity_mps", 5.0))

        self.path = None
        self.velocity_limit = None

        input_path_topic = rospy.get_param('~input_path_topic', '~input/path')
        max_velocity_topic = rospy.get_param('~max_velocity_topic', '/planning/scenario_planning/max_velocity')
        output_path_topic = rospy.get_param('~output_path_topic', '~output/path')

        self.sub_path = rospy.Subscriber(input_path_topic, PathWithLaneId, self.cb_path)
        if max_velocity_topic:
            self.sub_velocity_limit = rospy.Subscriber(max_velocity_topic, VelocityLimit, self.cb_velocity_limit)
        self.pub_path = rospy.Publisher(output_path_topic, PathWithLaneId, queue_size=1)
        self.pub_path = rospy.Publisher(
            "/output/motion_velocity_path", PathWithLaneId, queue_size=1
        )

        rospy.loginfo("VelocityPlanner initialized")

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.publish_path()

    def cb_velocity_limit(self, msg: VelocityLimit):
        self.velocity_limit = msg
        self.publish_path()

    def current_target_velocity(self):
        if self.velocity_limit is not None and self.velocity_limit.max_velocity > 0.0:
            return min(self.default_velocity, self.velocity_limit.max_velocity)
        return self.default_velocity

    def publish_path(self):
        if not self.path:
            return

        out = deepcopy(self.path)
        target_velocity = self.current_target_velocity()
        for point in out.points:
            try:
                original_velocity = float(point.point.longitudinal_velocity_mps)
            except Exception:
                original_velocity = 0.0

            if original_velocity > 0.0:
                point.point.longitudinal_velocity_mps = min(original_velocity, target_velocity)
            else:
                point.point.longitudinal_velocity_mps = target_velocity

        self.pub_path.publish(out)

    def run(self):
        rate = rospy.Rate(self.planning_hz)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("velocity_planner")
    planner = VelocityPlanner()
    planner.run()
