#pragma once
#include <ros/ros.h>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_internal_debug_msgs/StringStamped.h>
#include <geometry_msgs/Pose.h>
#include <autoware/objects_of_interest_marker_interface/marker_data.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <limits>

namespace autoware::behavior_velocity_planner {
using geometry_msgs::Pose;
using geometry_msgs::Point;
using autoware::objects_of_interest_marker_interface::ColorName;
struct CrosswalkRos1Clock { ros::Time now() const { return ros::Time::now(); } };
namespace crosswalk_ros1 {
template<class T> T require_parameter(const ros::NodeHandle & node, const std::string & name)
{
  return autoware_utils_rclcpp::detail::strict_get<T>(
    node, autoware_utils_rclcpp::to_ros1_parameter_name(name));
}
template<> inline float require_parameter<float>(const ros::NodeHandle & node, const std::string & name)
{
  const double value = require_parameter<double>(node, name);
  if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
    throw std::runtime_error("nonrepresentable float parameter: " + name);
  }
  return static_cast<float>(value);
}
}
}
#ifndef RCLCPP_DEBUG
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#endif
#ifndef RCLCPP_INFO
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#endif
#ifndef RCLCPP_INFO_EXPRESSION
#define RCLCPP_INFO_EXPRESSION(logger, condition, ...) ROS_INFO_COND(condition, __VA_ARGS__)
#endif
#ifndef RCLCPP_INFO_THROTTLE
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) ROS_INFO_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_WARN_THROTTLE
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) ROS_WARN_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_ERROR_STREAM_THROTTLE
#define RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, ms, message) ROS_ERROR_STREAM_THROTTLE((ms) / 1000.0, message)
#endif
