#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ros/console.h>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <thread>
#include <cstdarg>
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

#include <autoware_utils_rclcpp/parameter.hpp>
#include <builtin_interfaces/Time.h>
#include <visualization_msgs/MarkerArray.h>
#ifndef RCLCPP_ERROR_THROTTLE
#define RCLCPP_ERROR_THROTTLE(logger, clock, milliseconds, ...) ROS_ERROR_THROTTLE((milliseconds)/1000.0,__VA_ARGS__)
#endif
namespace autoware::behavior_path_planner {
using visualization_msgs::MarkerArray;
inline ros::Time side_shift_ros1_time(const builtin_interfaces::Time & stamp) {
  if(stamp.sec<0 || stamp.nanosec>=1000000000U) throw std::invalid_argument("Invalid lateral offset timestamp");
  return ros::Time(static_cast<uint32_t>(stamp.sec),stamp.nanosec);
}
}
