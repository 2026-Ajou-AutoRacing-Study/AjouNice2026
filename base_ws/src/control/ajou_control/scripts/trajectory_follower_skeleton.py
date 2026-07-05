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
            # TODO: 현재 ego 위치에서 가장 가까운 경로 점을 찾습니다.
            # TODO: ego pose와 기준 경로 사이의 횡방향 오차를 계산합니다.
            # TODO: ego yaw와 경로 yaw 사이의 heading 오차를 계산합니다.
            # TODO: Pure Pursuit 조향 제어기를 구현합니다.
            # TODO: 종방향 PI 속도 제어기를 구현합니다.
            # TODO: 제어기 출력을 MORAI CtrlCmd 필드로 변환합니다.
            self.ctrl_pub.publish(self.build_safe_command())
            rospy.loginfo_throttle(
                1.0,
                "Trajectory follower skeleton: 안전 제동 명령 발행 중. path_ready=%s ego_ready=%s target_speed=%.2f",
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
