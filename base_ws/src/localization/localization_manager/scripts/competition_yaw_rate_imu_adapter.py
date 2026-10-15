#!/usr/bin/env python3

"""Build an experimental IMU measurement with Competition yaw-rate.

MORAI Competition Status reports body yaw-rate in clockwise degrees/second,
whereas ROS ``sensor_msgs/Imu`` expects counter-clockwise radians/second in
``base_link``. The Competition packet and MORAI IMU are derived from the same
simulator state, so this node replaces (rather than fuses) IMU yaw-rate. The
most recent IMU sample supplies acceleration and the Competition message owns
the output timestamp.
"""

import copy
import math

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu


def competition_yaw_rate_radps(native_yaw_rate, scale=-math.pi / 180.0):
    """Convert native clockwise degrees/s to configured ROS yaw-rate."""
    return float(native_yaw_rate) * float(scale)


class CompetitionYawRateImuAdapter:
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/sensor/imu")
        self.competition_twist_topic = rospy.get_param(
            "~competition_twist_topic", "/competition/raw_twist"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/competition_imu"
        )
        self.max_imu_age_sec = float(rospy.get_param("~max_imu_age_sec", 0.05))
        self.yaw_rate_scale = float(
            rospy.get_param("~yaw_rate_scale", -math.pi / 180.0)
        )
        if not math.isfinite(self.max_imu_age_sec) or self.max_imu_age_sec <= 0.0:
            raise rospy.ROSInitException("~max_imu_age_sec must be positive")
        if not math.isfinite(self.yaw_rate_scale) or self.yaw_rate_scale == 0.0:
            raise rospy.ROSInitException("~yaw_rate_scale must be finite and non-zero")

        self.latest_imu = None
        self.last_output_stamp = None
        self.publisher = rospy.Publisher(self.output_topic, Imu, queue_size=100)
        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=200
        )
        self.twist_subscriber = rospy.Subscriber(
            self.competition_twist_topic,
            TwistStamped,
            self.twist_callback,
            queue_size=200,
        )
        rospy.loginfo(
            "competition_yaw_rate_imu_adapter: %s + %s -> %s; scale=%+.9f",
            self.imu_topic,
            self.competition_twist_topic,
            self.output_topic,
            self.yaw_rate_scale,
        )

    def imu_callback(self, message):
        if not message.header.stamp.is_zero():
            self.latest_imu = message

    def twist_callback(self, message):
        stamp = message.header.stamp
        if stamp.is_zero() or self.latest_imu is None:
            return
        if self.last_output_stamp is not None and stamp <= self.last_output_stamp:
            rospy.logwarn_throttle(
                2.0, "Rejecting duplicate Competition yaw-rate timestamp"
            )
            return
        age_sec = stamp.to_sec() - self.latest_imu.header.stamp.to_sec()
        if abs(age_sec) > self.max_imu_age_sec:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for co-time IMU acceleration: signed age %.3fs exceeds %.3fs",
                age_sec,
                self.max_imu_age_sec,
            )
            return

        yaw_rate = competition_yaw_rate_radps(
            message.twist.angular.z, self.yaw_rate_scale
        )
        if not math.isfinite(yaw_rate):
            rospy.logwarn_throttle(2.0, "Rejecting non-finite Competition yaw-rate")
            return

        output = copy.deepcopy(self.latest_imu)
        output.header.seq = message.header.seq
        output.header.stamp = stamp
        output.header.frame_id = "base_link"
        output.angular_velocity.x = 0.0
        output.angular_velocity.y = 0.0
        output.angular_velocity.z = yaw_rate
        output.angular_velocity_covariance = (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.publisher.publish(output)
        self.last_output_stamp = stamp


def main():
    rospy.init_node("competition_yaw_rate_imu_adapter")
    CompetitionYawRateImuAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
