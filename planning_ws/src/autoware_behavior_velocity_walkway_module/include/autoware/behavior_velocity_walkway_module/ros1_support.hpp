#pragma once
#include <ros/ros.h>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_internal_debug_msgs/StringStamped.h>
#include <autoware_internal_debug_msgs/Float64MultiArrayStamped.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>
#include <autoware/objects_of_interest_marker_interface/marker_data.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <charconv>

namespace autoware::behavior_velocity_planner {
using autoware_internal_planning_msgs::PathWithLaneId;
using ros::Time;
using autoware_utils::Polygon2d;
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using geometry_msgs::Pose;
using geometry_msgs::Point;
using autoware::objects_of_interest_marker_interface::ColorName;
struct WalkwayRos1Clock { ros::Time now() const { return ros::Time::now(); } };
namespace walkway_ros1 {
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
template<> inline std::vector<int64_t> require_parameter<std::vector<int64_t>>(
  const ros::NodeHandle & node, const std::string & name)
{
  // XML-RPC has only 32-bit integer scalars; decimal strings preserve full
  // lanelet int64 IDs. Never route IDs through floating-point conversion.
  XmlRpc::XmlRpcValue values;
  if (!node.getParam(autoware_utils_rclcpp::to_ros1_parameter_name(name), values) ||
      values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("required integer-array parameter: " + name);
  }
  std::vector<int64_t> result;
  for (int i = 0; i < values.size(); ++i) {
    auto & value = values[i];
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result.push_back(static_cast<int>(value));
    } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
      const std::string text = static_cast<std::string>(value);
      int64_t integer{};
      const auto parsed = std::from_chars(text.data(), text.data() + text.size(), integer);
      if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
        throw std::runtime_error("invalid int64 parameter element: " + name);
      }
      result.push_back(integer);
    } else {
      throw std::runtime_error("noninteger parameter element: " + name);
    }
  }
  return result;
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

namespace autoware::behavior_velocity_planner::walkway_ros1 {
template<class T> ros::Duration duration(const T & value) { return ros::Duration(value.sec, value.nanosec); }
inline ros::Duration duration(double value) { return ros::Duration(value); }
inline ros::Duration duration(const ros::Duration & value) { return value; }
}

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <geometry_msgs/Point32.h>
namespace autoware::behavior_velocity_planner::walkway_ros1 {
inline geometry_msgs::Point32 point32(double x, double y, double z) {
  geometry_msgs::Point32 result; result.x = x; result.y = y; result.z = z; return result;
}
}

#include <std_msgs/String.h>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
namespace autoware::behavior_velocity_planner::walkway_ros1 {
template<class T> void stamp(T & out, const ros::Time & time) { out.sec = time.sec; out.nanosec = time.nsec; }
inline void stamp(ros::Time & out, const ros::Time & time) { out = time; }
}
#ifndef RCLCPP_ERROR_THROTTLE
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) ROS_ERROR_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG_SKIPFIRST_THROTTLE
#define RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger, clock, ms, ...) ROS_DEBUG_DELAYED_THROTTLE((ms) / 1000.0, __VA_ARGS__)
#endif
#ifndef RCLCPP_ERROR
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN_ONCE
#define RCLCPP_WARN_ONCE(logger, ...) ROS_WARN_ONCE(__VA_ARGS__)
#endif

namespace autoware::behavior_velocity_planner::walkway_ros1 {
template<class Machine, class State, class Clock>
void setStateWithMarginTime(Machine & machine, State requested, const std::string &, Clock & clock) {
  machine.setStateWithMarginTime(requested, clock.now());
}
}
