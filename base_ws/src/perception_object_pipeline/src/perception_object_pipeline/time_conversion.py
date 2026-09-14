"""Conversions for duration fields in the ROS 1-ported Autoware messages."""

from builtin_interfaces.msg import Duration


def builtin_duration_from_seconds(seconds):
    """Return the generated message used by ported ROS 2 Duration fields."""
    total_nanoseconds = int(round(float(seconds) * 1_000_000_000))
    sec, nanosec = divmod(total_nanoseconds, 1_000_000_000)
    return Duration(sec=sec, nanosec=nanosec)


def duration_to_seconds(value):
    """Convert ROS 1 or ROS 2-style Duration objects to seconds."""
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    sec = getattr(value, "sec", getattr(value, "secs", 0))
    nanosec = getattr(value, "nanosec", getattr(value, "nsecs", 0))
    return float(sec) + float(nanosec) * 1.0e-9
