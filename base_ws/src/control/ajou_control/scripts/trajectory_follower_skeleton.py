#!/usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path
from std_msgs.msg import Float32


class TrajectoryFollowerSkeleton:
    def __init__(self):
        self.reference_path = None
        self.target_speed_mps = 0.0
        self.ego_status = None

        rospy.Subscriber("/reference_path", Path, self.reference_path_callback, queue_size=1)
        rospy.Subscriber("/target_speed", Float32, self.target_speed_callback, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback, queue_size=1)

        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

    def reference_path_callback(self, msg):
        self.reference_path = msg

    def target_speed_callback(self, msg):
        self.target_speed_mps = msg.data

    def ego_status_callback(self, msg):
        self.ego_status = msg

    def build_safe_command(self):
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = 0.0
        cmd.brake = 1.0
        cmd.front_steer = 0.0
        return cmd

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # TODO: Find nearest path point to current ego position.
            # TODO: Calculate lateral error from ego pose to reference path.
            # TODO: Calculate heading error between ego yaw and path yaw.
            # TODO: Implement Pure Pursuit steering controller.
            # TODO: Implement longitudinal PI speed controller.
            # TODO: Convert controller outputs to MORAI CtrlCmd fields.
            self.ctrl_pub.publish(self.build_safe_command())
            rospy.loginfo_throttle(
                1.0,
                "Trajectory follower skeleton publishing safe brake command. path_ready=%s ego_ready=%s target_speed=%.2f",
                self.reference_path is not None,
                self.ego_status is not None,
                self.target_speed_mps,
            )
            rate.sleep()


def main():
    rospy.init_node("trajectory_follower_skeleton")
    TrajectoryFollowerSkeleton().run()


if __name__ == "__main__":
    main()

