#!/usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path


class TrackingValidatorSkeleton:
    def __init__(self):
        self.reference_path = None
        self.ego_status = None
        self.ctrl_cmd = None

        rospy.Subscriber("/reference_path", Path, self.reference_path_callback, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback, queue_size=1)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.ctrl_cmd_callback, queue_size=1)

    def reference_path_callback(self, msg):
        self.reference_path = msg

    def ego_status_callback(self, msg):
        self.ego_status = msg

    def ctrl_cmd_callback(self, msg):
        self.ctrl_cmd = msg

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # TODO: 검증 구간의 횡방향 RMS 오차를 계산합니다.
            # TODO: 검증 구간의 heading RMS 오차를 계산합니다.
            # TODO: /target_speed 또는 궤적 metadata 기준 속도 오차를 계산합니다.
            # TODO: CtrlCmd.front_steer 제한값 기준 조향 포화 여부를 확인합니다.
            # TODO: 오프라인 제어기 검증용 CSV 로그를 추가합니다.
            lateral_rms_error = 0.0
            heading_rms_error = 0.0
            speed_error = 0.0
            steering_saturated = False

            rospy.loginfo_throttle(
                1.0,
                "Tracking validator skeleton: path_ready=%s ego_ready=%s cmd_ready=%s lateral_rms=%.3f heading_rms=%.3f speed_error=%.3f steering_saturated=%s",
                self.reference_path is not None,
                self.ego_status is not None,
                self.ctrl_cmd is not None,
                lateral_rms_error,
                heading_rms_error,
                speed_error,
                steering_saturated,
            )
            rate.sleep()


def main():
    rospy.init_node("tracking_validator_skeleton")
    TrackingValidatorSkeleton().run()


if __name__ == "__main__":
    main()
