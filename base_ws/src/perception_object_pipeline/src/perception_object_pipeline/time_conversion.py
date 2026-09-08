"""Conversions between ROS 1 time objects and ported ROS 2 builtin messages."""

import rospy
from builtin_interfaces.msg import Duration


def builtin_duration_from_seconds(seconds):
    """Return a normalized builtin_interfaces/Duration from seconds."""
    value = rospy.Duration.from_sec(float(seconds))
    return Duration(sec=value.secs, nanosec=value.nsecs)


def duration_to_seconds(value):
    """Convert builtin_interfaces/Duration to floating-point seconds."""
    return float(value.sec) + float(value.nanosec) * 1.0e-9
