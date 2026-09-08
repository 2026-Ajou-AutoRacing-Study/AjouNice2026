#ifndef AUTOWARE_UTILS_RCLCPP__PARAMETER_HPP_
#define AUTOWARE_UTILS_RCLCPP__PARAMETER_HPP_

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <chrono>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware_utils_rclcpp
{
inline std::string to_ros1_parameter_name(std::string name)
{
  std::replace(name.begin(), name.end(), '.', '/');
  return name;
}

namespace detail
{
template <class T> struct xmlrpc_type { static constexpr auto value = XmlRpc::XmlRpcValue::TypeInvalid; };
template <> struct xmlrpc_type<bool> { static constexpr auto value = XmlRpc::XmlRpcValue::TypeBoolean; };
template <> struct xmlrpc_type<int> { static constexpr auto value = XmlRpc::XmlRpcValue::TypeInt; };
template <> struct xmlrpc_type<double> { static constexpr auto value = XmlRpc::XmlRpcValue::TypeDouble; };
template <> struct xmlrpc_type<std::string> { static constexpr auto value = XmlRpc::XmlRpcValue::TypeString; };
template <class T> struct xmlrpc_type<std::vector<T>> { static constexpr auto value = XmlRpc::XmlRpcValue::TypeArray; };

template <class T>
T strict_get(const ros::NodeHandle & node, const std::string & name)
{
  static_assert(xmlrpc_type<T>::value != XmlRpc::XmlRpcValue::TypeInvalid,
    "unsupported strict ROS1 parameter type");
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(name, raw) || raw.getType() != xmlrpc_type<T>::value) {
    throw std::runtime_error("parameter type mismatch: " + name);
  }
  T value{};
  if (!node.getParam(name, value)) { throw std::runtime_error("parameter decode mismatch: " + name); }
  return value;
}
}  // namespace detail

template <class T>
T get_or_declare_parameter(const ros::NodeHandle & node, const std::string & name)
{
  const auto ros1_name = to_ros1_parameter_name(name);
  if (!node.hasParam(ros1_name)) { return T{}; }
  return detail::strict_get<T>(node, ros1_name);
}

template <class T>
bool update_param(const ros::NodeHandle & node, const std::string & name, T & value)
{
  const auto ros1_name = to_ros1_parameter_name(name);
  if (!node.hasParam(ros1_name)) { return false; }
  value = detail::strict_get<T>(node, ros1_name);
  return true;
}

template <class T>
[[deprecated]] T wait_for_param(
  const ros::NodeHandle * node, const std::string & remote_node_name, const std::string & param_name)
{
  if (node == nullptr) { throw std::invalid_argument("wait_for_param needs NodeHandle"); }
  const std::string resolved = remote_node_name + "/" + param_name;
  T value{};
  while (ros::ok()) {
    if (node->hasParam(resolved)) { return detail::strict_get<T>(*node, resolved); }
    ros::WallDuration(1.0).sleep();
  }
  return T{};
}
}  // namespace autoware_utils_rclcpp

#endif
