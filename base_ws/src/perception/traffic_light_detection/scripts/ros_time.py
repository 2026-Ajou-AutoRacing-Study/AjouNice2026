"""ROS 1 time conversion for ported ROS 2 builtin message fields."""

from builtin_interfaces.msg import Time


def builtin_time_from_ros(value):
    return Time(sec=value.secs, nanosec=value.nsecs)
