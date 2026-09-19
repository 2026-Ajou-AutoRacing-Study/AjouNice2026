#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ros/console.h>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <thread>
#include <cstdarg>
#ifndef RCLCPP_FATAL_STREAM
#define RCLCPP_FATAL_STREAM(logger, message) ROS_FATAL_STREAM(message)
#endif
#ifndef RCLCPP_INFO
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#endif
#ifndef RCLCPP_FATAL
#define RCLCPP_FATAL(logger, ...) ROS_FATAL(__VA_ARGS__)
#endif
#ifndef RCLCPP_INFO_THROTTLE
#define RCLCPP_INFO_THROTTLE(logger, clock, milliseconds, ...) ROS_INFO_THROTTLE((milliseconds) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG_THROTTLE
#define RCLCPP_DEBUG_THROTTLE(logger, clock, milliseconds, ...) ROS_DEBUG_THROTTLE((milliseconds) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG_EXPRESSION
#define RCLCPP_DEBUG_EXPRESSION(logger, condition, ...) ROS_DEBUG_COND(condition, __VA_ARGS__)
#endif
