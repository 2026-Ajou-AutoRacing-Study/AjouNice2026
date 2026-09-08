#pragma once

#include <ros/ros.h>

// The algorithm is retained verbatim where possible.  These macros only map
// ROS2 logging call sites to ROS1 and deliberately do not emulate a ROS2 node
// or executor.
#undef RCLCPP_DEBUG
#undef RCLCPP_INFO
#undef RCLCPP_WARN
#undef RCLCPP_ERROR
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
