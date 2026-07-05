#!/usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd


def build_command():
    cmd = CtrlCmd()
    cmd.longlCmdType = int(rospy.get_param("~longlCmdType", 1))
    cmd.accel = float(rospy.get_param("~accel", 0.0))
    cmd.brake = float(rospy.get_param("~brake", 1.0))
    cmd.front_steer = float(rospy.get_param("~front_steer", 0.0))
    return cmd


def main():
    rospy.init_node("simple_morai_cmd")
    cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        cmd = build_command()
        cmd_pub.publish(cmd)
        rospy.loginfo_throttle(
            1.0,
            "CtrlCmd 확인: longlCmdType=%d accel=%.3f brake=%.3f front_steer=%.3f",
            cmd.longlCmdType,
            cmd.accel,
            cmd.brake,
            cmd.front_steer,
        )
        rate.sleep()


if __name__ == "__main__":
    main()
