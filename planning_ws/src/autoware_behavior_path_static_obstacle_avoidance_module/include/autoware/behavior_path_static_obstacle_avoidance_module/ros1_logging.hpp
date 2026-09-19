#pragma once

#include <ros/ros.h>

// Complete the logger facade used by the selected BPP host. ROS2 throttle
// periods are milliseconds; roscpp throttle periods are seconds.
#ifndef RCLCPP_DEBUG_STREAM
#define RCLCPP_DEBUG_STREAM(logger, expression) ROS_DEBUG_STREAM(expression)
#endif
#ifndef RCLCPP_INFO_THROTTLE
#define RCLCPP_INFO_THROTTLE(logger, clock, period_ms, ...) \
  ROS_INFO_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG_THROTTLE
#define RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, ...) \
  ROS_DEBUG_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)
#endif
