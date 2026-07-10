#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_planning_msgs.msg import LaneletRoute, PoseWithUuidStamped
from tier4_planning_msgs.msg import RerouteAvailability

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import PathModuleBase, apply_goal_shift, path_length


class GoalPlannerBPP(PathModuleBase):
    def __init__(self):
        self.route = None

        self.goal_lateral_offset = float(rospy.get_param("~goal_lateral_offset", 0.0))
        self.pull_over_distance = float(rospy.get_param("~pull_over_distance", 20.0))
        self.stop_at_goal = bool(rospy.get_param("~stop_at_goal", True))
        self.allow_goal_modification = bool(rospy.get_param("~allow_goal_modification", True))

        route_topic = rospy.get_param("~route_topic", "~input/route")
        rospy.Subscriber(route_topic, LaneletRoute, self.route_callback, queue_size=1)

        self.modified_goal_pub = rospy.Publisher(
            "~output/modified_goal", PoseWithUuidStamped, queue_size=1
        )
        self.reroute_pub = rospy.Publisher(
            "~output/is_reroute_available", RerouteAvailability, queue_size=1
        )

        super().__init__("goal_planner_bpp")

    def route_callback(self, msg):
        self.route = msg

    def publish_side_outputs(self, path, stamp):
        modified_goal = PoseWithUuidStamped()
        modified_goal.header.stamp = stamp
        modified_goal.header.frame_id = path.header.frame_id or self.frame_id
        if path.points:
            modified_goal.pose = path.points[-1].point.pose
        elif self.route is not None:
            modified_goal.pose = self.route.goal_pose
        if self.route is not None:
            modified_goal.uuid = self.route.uuid
        self.modified_goal_pub.publish(modified_goal)

        reroute = RerouteAvailability()
        reroute.stamp = stamp
        reroute.availability = self.allow_goal_modification
        self.reroute_pub.publish(reroute)

    def process_path(self, path, stamp):
        output = apply_goal_shift(path, self.goal_lateral_offset, self.pull_over_distance)

        if self.stop_at_goal and output.points:
            output.points[-1].point.longitudinal_velocity_mps = 0.0
            output.points[-1].point.is_final = True

        self.publish_side_outputs(output, stamp)

        if abs(self.goal_lateral_offset) > 1.0e-6:
            return output, "goal_planner: pull-over offset {:.2f} m over {:.2f} m".format(
                self.goal_lateral_offset, min(self.pull_over_distance, path_length(output))
            )
        return output, "goal_planner: fixed goal pass-through"


def main():
    rospy.init_node("goal_planner_bpp")
    GoalPlannerBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
