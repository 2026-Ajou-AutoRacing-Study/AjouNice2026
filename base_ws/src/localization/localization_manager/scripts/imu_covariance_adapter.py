#!/usr/bin/env python3

import copy
import math

import rospy
from sensor_msgs.msg import Imu


DEFAULT_NOISE_NS = "/localization_manager/providers/gnss_imu/noise"


def diagonal_covariance(variance):
    return [
        variance,
        0.0,
        0.0,
        0.0,
        variance,
        0.0,
        0.0,
        0.0,
        variance,
    ]


def covariance_is_valid(values):
    if len(values) != 9 or not all(math.isfinite(value) for value in values):
        return False
    if values[0] < 0.0:
        return False
    diagonal = (values[0], values[4], values[8])
    return all(value >= 0.0 for value in diagonal) and any(
        value > 0.0 for value in diagonal
    )


class ImuCovarianceAdapter:
    """Normalize MORAI IMU metadata without modifying its measured values."""

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/sensor/imu")
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/imu"
        )
        self.output_frame = rospy.get_param("~output_frame", "")
        self.noise_ns = rospy.get_param("~noise_namespace", DEFAULT_NOISE_NS)
        self.preserve_valid_covariance = bool(
            rospy.get_param("~preserve_valid_covariance", True)
        )
        self.reject_duplicate_stamps = bool(
            rospy.get_param("~reject_duplicate_stamps", True)
        )

        orientation_stddev = self.positive_param(
            "imu/orientation_stddev_rad"
        )
        angular_velocity_stddev = self.positive_param(
            "imu/angular_velocity_stddev_radps"
        )
        linear_acceleration_stddev = self.positive_param(
            "imu/linear_acceleration_stddev_mps2"
        )

        self.orientation_covariance = diagonal_covariance(
            orientation_stddev**2
        )
        self.angular_velocity_covariance = diagonal_covariance(
            angular_velocity_stddev**2
        )
        self.linear_acceleration_covariance = diagonal_covariance(
            linear_acceleration_stddev**2
        )
        self.last_stamp = None

        self.publisher = rospy.Publisher(
            self.output_topic, Imu, queue_size=100
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, Imu, self.imu_callback, queue_size=200
        )

        rospy.loginfo(
            "imu_covariance_adapter: %s -> %s; profile=%s",
            self.input_topic,
            self.output_topic,
            self.noise_ns,
        )

    def positive_param(self, relative_name):
        value = float(rospy.get_param(self.noise_ns + "/" + relative_name))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException(
                "{} must be a positive finite standard deviation".format(
                    self.noise_ns + "/" + relative_name
                )
            )
        return value

    @staticmethod
    def measurement_values(message):
        return (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )

    def imu_callback(self, message):
        if message.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Rejecting IMU with zero timestamp")
            return
        if (
            self.reject_duplicate_stamps
            and self.last_stamp is not None
            and message.header.stamp <= self.last_stamp
        ):
            rospy.logwarn_once(
                "Rejecting duplicate/out-of-order MORAI IMU; "
                "subsequent duplicates are suppressed"
            )
            return
        if not all(
            math.isfinite(value) for value in self.measurement_values(message)
        ):
            rospy.logwarn_throttle(2.0, "Rejecting IMU with non-finite value")
            return

        quaternion_norm = math.sqrt(
            message.orientation.x**2
            + message.orientation.y**2
            + message.orientation.z**2
            + message.orientation.w**2
        )
        if quaternion_norm < 1.0e-9:
            rospy.logwarn_throttle(2.0, "Rejecting IMU with invalid quaternion")
            return

        output = copy.deepcopy(message)
        if self.output_frame:
            output.header.frame_id = self.output_frame

        output.orientation.x /= quaternion_norm
        output.orientation.y /= quaternion_norm
        output.orientation.z /= quaternion_norm
        output.orientation.w /= quaternion_norm

        if not (
            self.preserve_valid_covariance
            and covariance_is_valid(message.orientation_covariance)
        ):
            output.orientation_covariance = self.orientation_covariance
        if not (
            self.preserve_valid_covariance
            and covariance_is_valid(message.angular_velocity_covariance)
        ):
            output.angular_velocity_covariance = (
                self.angular_velocity_covariance
            )
        if not (
            self.preserve_valid_covariance
            and covariance_is_valid(message.linear_acceleration_covariance)
        ):
            output.linear_acceleration_covariance = (
                self.linear_acceleration_covariance
            )

        self.last_stamp = message.header.stamp
        self.publisher.publish(output)


def main():
    rospy.init_node("imu_covariance_adapter")
    ImuCovarianceAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
