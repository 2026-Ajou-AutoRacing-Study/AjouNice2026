#pragma once

#include "autoware/path_smoother/ros1_compat.hpp"

#undef RCLCPP_DEBUG
#undef RCLCPP_INFO
#undef RCLCPP_WARN
#undef RCLCPP_ERROR
#undef RCLCPP_WARN_STREAM
#undef RCLCPP_WARN_THROTTLE
#undef RCLCPP_INFO_THROTTLE
#undef RCLCPP_INFO_SKIPFIRST_THROTTLE
#undef RCLCPP_INFO_EXPRESSION
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR_NAMED((logger).get_name().c_str(), __VA_ARGS__)
#define RCLCPP_WARN_STREAM(logger, args) ROS_WARN_STREAM_NAMED((logger).get_name().c_str(), args)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) ROS_WARN_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) ROS_INFO_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(logger, clock, ms, ...) ROS_INFO_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_EXPRESSION(logger, condition, ...) do { if (condition) { ROS_INFO_NAMED((logger).get_name().c_str(), __VA_ARGS__); } } while (0)
