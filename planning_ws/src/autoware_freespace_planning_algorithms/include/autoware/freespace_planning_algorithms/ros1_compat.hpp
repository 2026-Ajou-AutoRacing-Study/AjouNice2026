// ROS1 boundary for the formerly rclcpp-owned planner constructors.
#ifndef AUTOWARE_FREESPACE_PLANNING_ALGORITHMS__ROS1_COMPAT_HPP_
#define AUTOWARE_FREESPACE_PLANNING_ALGORITHMS__ROS1_COMPAT_HPP_

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <functional>
#include <stdexcept>
#include <string>

namespace autoware::freespace_planning_algorithms::ros1_compat
{
using ClockNow = std::function<ros::Time()>;

inline ClockNow ros_time_now()
{
  return []() { return ros::Time::now(); };
}

inline XmlRpc::XmlRpcValue require_value(const ros::NodeHandle & node, const std::string & key)
{
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(key, value)) {
    throw std::runtime_error("required parameter is missing: " + key);
  }
  return value;
}

template <typename T>
T require_param(const ros::NodeHandle & node, const std::string & key);

template <>
inline bool require_param<bool>(const ros::NodeHandle & node, const std::string & key)
{
  const auto value = require_value(node, key);
  if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::runtime_error("required bool parameter has wrong type: " + key);
  }
  return static_cast<bool>(value);
}

template <>
inline double require_param<double>(const ros::NodeHandle & node, const std::string & key)
{
  const auto value = require_value(node, key);
  if (value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    throw std::runtime_error("required double parameter has wrong type: " + key);
  }
  return static_cast<double>(value);
}

template <>
inline std::string require_param<std::string>(const ros::NodeHandle & node, const std::string & key)
{
  const auto value = require_value(node, key);
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("required string parameter has wrong type: " + key);
  }
  return static_cast<std::string>(value);
}
}  // namespace autoware::freespace_planning_algorithms::ros1_compat

#endif
