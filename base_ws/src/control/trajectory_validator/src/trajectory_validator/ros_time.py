"""Conversions for ROS 2 builtin time fields in the ported planning messages."""

import rospy
from builtin_interfaces.msg import Duration


def builtin_duration_from_seconds(seconds):
    value = rospy.Duration.from_sec(float(seconds))
    return Duration(sec=value.secs, nanosec=value.nsecs)
