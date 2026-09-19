#pragma once

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <xmlrpcpp/XmlRpcException.h>

#include <string>

namespace autoware_utils
{
// Preserve ROS2 update-only-if-present behavior at the ROS1 plugin boundary.
// A type mismatch is rejected instead of silently coercing operator input.
template <class T>
bool update_param(
  const autoware::motion_velocity_planner::ParameterMap & parameters, const std::string & name,
  T & value)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) return false;
  try {
    value = static_cast<T>(it->second);
    return true;
  } catch (const XmlRpc::XmlRpcException &) {
    return false;
  }
}
}  // namespace autoware_utils
