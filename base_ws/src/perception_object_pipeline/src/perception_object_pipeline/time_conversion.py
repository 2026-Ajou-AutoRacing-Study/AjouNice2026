"""Conversions for duration fields in the ROS 1-ported Autoware messages."""

import rospy


def builtin_duration_from_seconds(seconds):
    """Return the ROS 1 representation of a ported ROS 2 Duration field."""
    return rospy.Duration.from_sec(float(seconds))


def duration_to_seconds(value):
    """Convert ROS 1 or ROS 2-style Duration objects to seconds."""
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    sec = getattr(value, "sec", getattr(value, "secs", 0))
    nanosec = getattr(value, "nanosec", getattr(value, "nsecs", 0))
    return float(sec) + float(nanosec) * 1.0e-9
