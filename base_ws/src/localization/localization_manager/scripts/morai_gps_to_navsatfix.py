#!/usr/bin/env python3

import math

import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus


PARAM_NS = "/localization_manager"


def param(name, default=None):
    return rospy.get_param(PARAM_NS + "/" + name, default)


class MoraiGpsToNavSatFix:
    """Convert MORAI GPSMessage into the standard GNSS input expected by MSF."""

    def __init__(self):
        self.input_topic = param("topics/gps", "/sensor/gps")
        self.output_topic = param("topics/gnss_fix", "/localization/gnss/fix")
        self.horizontal_stddev = float(
            param("providers/fast_lio_gnss/gnss/horizontal_stddev_m", 0.5)
        )
        self.vertical_stddev = float(
            param("providers/fast_lio_gnss/gnss/vertical_stddev_m", 1.0)
        )
        self.reject_duplicate_stamps = bool(
            param("providers/fast_lio_gnss/gnss/reject_duplicate_stamps", True)
        )
        if self.horizontal_stddev <= 0.0 or self.vertical_stddev <= 0.0:
            raise rospy.ROSInitException("GNSS standard deviations must be positive")

        self.last_stamp = None
        self.publisher = rospy.Publisher(
            self.output_topic, NavSatFix, queue_size=100
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, GPSMessage, self.gps_callback, queue_size=100
        )
        rospy.loginfo(
            "morai_gps_to_navsatfix: %s -> %s; sigma_xy=%.3fm, sigma_z=%.3fm",
            self.input_topic,
            self.output_topic,
            self.horizontal_stddev,
            self.vertical_stddev,
        )

    def gps_callback(self, message):
        values = (message.latitude, message.longitude, message.altitude)
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, "Rejecting MORAI GPS with non-finite LLA")
            return
        if not (-90.0 <= message.latitude <= 90.0) or not (
            -180.0 <= message.longitude <= 180.0
        ):
            rospy.logwarn_throttle(2.0, "Rejecting MORAI GPS outside valid LLA range")
            return
        if message.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Rejecting MORAI GPS with zero timestamp")
            return
        if (
            self.reject_duplicate_stamps
            and self.last_stamp is not None
            and message.header.stamp <= self.last_stamp
        ):
            rospy.logwarn_throttle(2.0, "Rejecting duplicate/out-of-order MORAI GPS")
            return

        output = NavSatFix()
        output.header = message.header
        output.status.status = (
            NavSatStatus.STATUS_FIX
            if message.status >= 0
            else NavSatStatus.STATUS_NO_FIX
        )
        output.status.service = NavSatStatus.SERVICE_GPS
        output.latitude = message.latitude
        output.longitude = message.longitude
        output.altitude = message.altitude

        horizontal_variance = self.horizontal_stddev * self.horizontal_stddev
        vertical_variance = self.vertical_stddev * self.vertical_stddev
        output.position_covariance = [
            horizontal_variance,
            0.0,
            0.0,
            0.0,
            horizontal_variance,
            0.0,
            0.0,
            0.0,
            vertical_variance,
        ]
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.last_stamp = message.header.stamp
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_gps_to_navsatfix")
    MoraiGpsToNavSatFix()
    rospy.spin()


if __name__ == "__main__":
    main()
