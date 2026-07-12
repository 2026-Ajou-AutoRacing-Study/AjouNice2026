#!/usr/bin/env python3

import rospy


class VelocityPlanner:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        rospy.loginfo("VelocityPlanner initialized")

    def run(self):
        rate = rospy.Rate(self.planning_hz)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("velocity_planner")
    planner = VelocityPlanner()
    planner.run()
