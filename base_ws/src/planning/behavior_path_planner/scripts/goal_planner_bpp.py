#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_planning_msgs.msg import LaneletRoute, Path, PoseWithUuidStamped
from tier4_planning_msgs.msg import RerouteAvailability

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_goal_planner_tools import GoalPlannerScene
from bpp_path_tools import PathModuleBase


class GoalPlannerBPP(PathModuleBase):
    def __init__(self):
        self.route = None
        self.scene = GoalPlannerScene("~goal_planner")
        rospy.Subscriber(
            rospy.get_param("~route_topic", "~input/route"),
            LaneletRoute,
            self.route_callback,
            queue_size=1,
        )
        self.modified_goal_pub = rospy.Publisher(
            "~output/modified_goal", PoseWithUuidStamped, queue_size=1
        )
        self.reroute_pub = rospy.Publisher(
            "~output/is_reroute_available", RerouteAvailability, queue_size=1
        )
        self.reference_pub = rospy.Publisher(
            "~output/reference_path", Path, queue_size=1
        )
        super().__init__("goal_planner_bpp")

    def route_callback(self, message):
        self.route = message

    def publish_side_outputs(self, result, stamp):
        if result.modified_goal_pose is not None:
            modified = PoseWithUuidStamped()
            modified.header.stamp = stamp
            modified.header.frame_id = result.output_path.header.frame_id or self.frame_id
            modified.pose = result.modified_goal_pose
            if self.route is not None:
                modified.uuid = self.route.uuid
            self.modified_goal_pub.publish(modified)

        reroute = RerouteAvailability()
        reroute.stamp = stamp
        reroute.availability = self.scene.engine.parameters.allow_goal_modification
        self.reroute_pub.publish(reroute)

    def process_path(self, path, stamp):
        result = self.scene.process(path, self.route)
        inactive_reference = Path()
        inactive_reference.header.stamp = stamp
        inactive_reference.header.frame_id = path.header.frame_id or self.frame_id
        self.reference_pub.publish(inactive_reference)
        self.publish_side_outputs(result, stamp)
        return result.output_path, result.status


def main():
    rospy.init_node("goal_planner_bpp")
    GoalPlannerBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
