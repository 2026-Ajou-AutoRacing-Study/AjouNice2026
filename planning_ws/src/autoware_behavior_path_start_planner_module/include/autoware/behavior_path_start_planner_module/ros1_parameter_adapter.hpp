#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace autoware::behavior_path_planner::ros1_parameter_adapter
{
inline std::string toRos1Key(std::string ros2_key)
{
  std::replace(ros2_key.begin(), ros2_key.end(), '.', '/');
  return ros2_key;
}

template <class T>
T require(const ros::NodeHandle & node, const std::string & ros2_key)
{
  T value{};
  const auto ros1_key = toRos1Key(ros2_key);
  if (!node.getParam(ros1_key, value)) {
    throw std::runtime_error("missing required ROS1 parameter: " + ros2_key + " (" + ros1_key + ")");
  }
  return value;
}

// Retain the source call-site shape while intentionally changing its ROS2
// "declare if absent" behavior to ROS1 fail-closed retrieval.
template <class T>
T get_or_declare_parameter(const ros::NodeHandle & node, const std::string & key)
{
  return require<T>(node, key);
}
}  // namespace autoware::behavior_path_planner::ros1_parameter_adapter
