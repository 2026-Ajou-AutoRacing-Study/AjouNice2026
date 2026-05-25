#!/usr/bin/env python3

import rospy


def main():
    rospy.init_node("hello_node")

    name = rospy.get_param("~name", "AjouNice")
    rate_hz = rospy.get_param("~rate", 1.0)

    rospy.loginfo("hello_node started")
    rospy.loginfo("name: %s", name)
    rospy.loginfo("rate: %.2f Hz", rate_hz)

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello, %s! ROS1 Python node is alive.", name)
        rate.sleep()


if __name__ == "__main__":
    main()