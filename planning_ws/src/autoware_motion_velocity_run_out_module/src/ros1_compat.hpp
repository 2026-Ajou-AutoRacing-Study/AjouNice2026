#pragma once

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <xmlrpcpp/XmlRpcException.h>

#include <string>
#include <vector>

namespace autoware_utils
{
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

template <class T>
bool update_param(
  const autoware::motion_velocity_planner::ParameterMap & parameters, const std::string & name,
  std::vector<T> & value)
{
  const auto it = parameters.find(name);
  if (it == parameters.end() || it->second.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;
  try {
    std::vector<T> updated;
    updated.reserve(it->second.size());
    for (int i = 0; i < it->second.size(); ++i) updated.push_back(static_cast<T>(it->second[i]));
    value = std::move(updated);
    return true;
  } catch (const XmlRpc::XmlRpcException &) {
    return false;
  }
}
}  // namespace autoware_utils
