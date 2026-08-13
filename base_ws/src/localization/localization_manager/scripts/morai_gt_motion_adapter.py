#!/usr/bin/env python3

import math

import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import TwistWithCovarianceStamped
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import Imu


DEFAULT_NOISE_NS = "/localization_manager/providers/gnss_imu_gt/noise"


def normalized_frame(frame):
    return frame.lstrip("/")


def positive_finite_param(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise rospy.ROSInitException(
            "{} must be a positive finite value".format(name)
        )
    return value


class MoraiGtMotionAdapter:
    """Expose MORAI GT heading and longitudinal speed as EKF measurements."""

    def __init__(self):
        self.source = str(rospy.get_param("~source", "ego")).strip().lower()
        if self.source not in ("ego", "competition"):
            raise rospy.ROSInitException(
                "Unsupported GT source '{}'; expected ego or competition".format(
                    self.source
                )
            )

        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.competition_topic = rospy.get_param(
            "~competition_topic", "/Competition_topic"
        )
        self.input_topic = (
            self.ego_topic
            if self.source == "ego"
            else self.competition_topic
        )
        self.orientation_topic = rospy.get_param(
            "~orientation_topic", "/localization/gt/orientation"
        )
        self.twist_topic = rospy.get_param(
            "~twist_topic", "/localization/gt/twist"
        )

        self.map_frame = normalized_frame(
            rospy.get_param("~map_frame", "map")
        )
        self.source_frame = normalized_frame(
            rospy.get_param("~source_frame", "morai_map")
        )
        self.base_frame = normalized_frame(
            rospy.get_param("~base_frame", "base_link")
        )
        self.tf_timeout = positive_finite_param("~tf_timeout_sec", 0.05)

        heading_unit = str(
            rospy.get_param("~heading_unit", "degrees")
        ).strip().lower()
        if heading_unit not in ("degrees", "radians"):
            raise rospy.ROSInitException(
                "Unsupported heading unit '{}'; expected degrees or radians".format(
                    heading_unit
                )
            )
        self.heading_in_degrees = heading_unit == "degrees"
        self.reject_duplicate_stamps = bool(
            rospy.get_param("~reject_duplicate_stamps", True)
        )
        correction_ns = "/localization_manager/providers/gnss_imu_gt"
        default_correction_enabled = bool(
            rospy.get_param(
                correction_ns + "/use_heading_correction", True
            )
        )
        default_correction_deg = float(
            rospy.get_param(
                correction_ns + "/heading_yaw_offset_deg", -1.347
            )
        )
        self.use_heading_correction = bool(
            rospy.get_param(
                "~use_heading_correction", default_correction_enabled
            )
        )
        self.heading_yaw_offset_deg = float(
            rospy.get_param(
                "~heading_yaw_offset_deg", default_correction_deg
            )
        )
        if not math.isfinite(self.heading_yaw_offset_deg):
            raise rospy.ROSInitException(
                "~heading_yaw_offset_deg must be finite"
            )
        self.heading_yaw_offset_rad = math.radians(
            self.heading_yaw_offset_deg
        )

        noise_ns = rospy.get_param("~noise_namespace", DEFAULT_NOISE_NS)
        heading_stddev = positive_finite_param(
            noise_ns + "/gt/heading_stddev_rad", math.radians(0.1)
        )
        longitudinal_velocity_stddev = positive_finite_param(
            noise_ns + "/gt/longitudinal_velocity_stddev_mps", 0.05
        )

        self.orientation_covariance = [0.0] * 9
        self.orientation_covariance[0] = 1.0e6
        self.orientation_covariance[4] = 1.0e6
        self.orientation_covariance[8] = heading_stddev**2

        self.twist_covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            self.twist_covariance[index] = 1.0e6
        self.twist_covariance[0] = longitudinal_velocity_stddev**2

        self.last_stamp = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.orientation_publisher = rospy.Publisher(
            self.orientation_topic, Imu, queue_size=100
        )
        self.twist_publisher = rospy.Publisher(
            self.twist_topic, TwistWithCovarianceStamped, queue_size=100
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            EgoVehicleStatus,
            self.motion_callback,
            queue_size=200,
        )

        rospy.loginfo(
            "morai_gt_motion_adapter source=%s: %s -> [%s, %s], "
            "heading correction=%s (%+.3f deg)",
            self.source,
            self.input_topic,
            self.orientation_topic,
            self.twist_topic,
            self.use_heading_correction,
            self.heading_yaw_offset_deg,
        )

    def map_from_source_quaternion(self):
        if self.source_frame == self.map_frame:
            return (0.0, 0.0, 0.0, 1.0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.source_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for GT heading TF %s -> %s: %s",
                self.source_frame,
                self.map_frame,
                str(error),
            )
            return None
        rotation = transform.transform.rotation
        return (rotation.x, rotation.y, rotation.z, rotation.w)

    def motion_callback(self, message):
        stamp = message.header.stamp
        if stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Rejecting GT motion with zero timestamp")
            return
        if (
            self.reject_duplicate_stamps
            and self.last_stamp is not None
            and stamp <= self.last_stamp
        ):
            rospy.logwarn_once(
                "Rejecting duplicate/out-of-order MORAI GT motion; "
                "subsequent duplicates are suppressed"
            )
            return

        heading = float(message.heading)
        velocity_x = float(message.velocity.x)
        if not math.isfinite(heading) or not math.isfinite(velocity_x):
            rospy.logwarn_throttle(
                2.0, "Rejecting GT motion with non-finite heading or velocity"
            )
            return

        yaw = math.radians(heading) if self.heading_in_degrees else heading
        if self.use_heading_correction:
            yaw += self.heading_yaw_offset_rad
        map_from_source = self.map_from_source_quaternion()
        if map_from_source is None:
            return
        source_from_base = tft.quaternion_from_euler(0.0, 0.0, yaw)
        map_from_base = tft.quaternion_multiply(
            map_from_source, source_from_base
        )
        quaternion_norm = math.sqrt(sum(value**2 for value in map_from_base))
        if quaternion_norm < 1.0e-9:
            rospy.logwarn_throttle(
                2.0, "Rejecting GT motion with invalid orientation"
            )
            return
        map_from_base = tuple(
            value / quaternion_norm for value in map_from_base
        )

        orientation = Imu()
        orientation.header.seq = message.header.seq
        orientation.header.stamp = stamp
        orientation.header.frame_id = self.base_frame
        orientation.orientation.x = map_from_base[0]
        orientation.orientation.y = map_from_base[1]
        orientation.orientation.z = map_from_base[2]
        orientation.orientation.w = map_from_base[3]
        orientation.orientation_covariance = self.orientation_covariance
        orientation.angular_velocity_covariance[0] = -1.0
        orientation.linear_acceleration_covariance[0] = -1.0

        twist = TwistWithCovarianceStamped()
        twist.header.seq = message.header.seq
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.twist.linear.x = velocity_x
        twist.twist.covariance = self.twist_covariance

        self.orientation_publisher.publish(orientation)
        self.twist_publisher.publish(twist)
        self.last_stamp = stamp


def main():
    rospy.init_node("morai_gt_motion_adapter")
    MoraiGtMotionAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
