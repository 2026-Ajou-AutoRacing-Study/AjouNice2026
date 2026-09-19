#pragma once
#include <autoware/motion_velocity_planner_common/utils.hpp>

#include "type_alias.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <ros/node_handle.h>
#include <xmlrpcpp/XmlRpcException.h>

#include <cstdint>
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

// Candidate-local compatibility spelling for residual source-level time and
// logging calls.  It deliberately maps to ROS1 primitives; it does not import
// or emulate a ROS2 executor.
namespace rclcpp
{
using Time = ros::Time;
}
#undef RCLCPP_DEBUG
#undef RCLCPP_INFO
#undef RCLCPP_WARN
#undef RCLCPP_ERROR
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)

// ROS2 parameter events carry typed Parameters.  At the ROS1 module boundary
// the manager supplies the equivalent typed XmlRpc map; retain update-only-if-
// present semantics and reject a mismatched value rather than silently
// coercing it.
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
}  // namespace autoware_utils

namespace autoware::motion_velocity_planner::utils
{
// R019 projects dotted ROS2 names to ROS1 private parameter paths.  The
// existing facade preserves strict type checking once this path is resolved.
std::string ros1_parameter_name(const std::string & name);
std::vector<uint8_t> get_target_object_type(ros::NodeHandle & node, const std::string & param_prefix);
visualization_msgs::Marker get_object_marker(
  const geometry_msgs::Pose & pose, size_t index, const std::string & ns, double r, double g,
  double b);
double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info);
double calc_object_possible_max_dist_from_center(const Shape & shape);


}  // namespace autoware::motion_velocity_planner::utils
