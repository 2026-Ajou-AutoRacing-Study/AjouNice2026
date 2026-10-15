#pragma once
#include "autoware/motion_utils/trajectory/acceleration_limits.hpp"
#include <ros/ros.h>

namespace autoware::motion_utils
{
// One immutable, launch-loaded table shared by planning nodes. Restart planning
// to reload; never query the parameter server inside a trajectory/QP loop.
inline AccelerationLimits loadPlanningAccelerationLimits()
{
  XmlRpc::XmlRpcValue rows;
  if (!ros::param::get("/planning/longitudinal_acceleration_limits/speed_acceleration_map", rows))
    return AccelerationLimits{};  // Standalone packages retain their scalar policy.
  if (rows.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::invalid_argument("Planning acceleration map must be an array of [m/s, m/s2]");
  AccelerationLimits::Map map;
  const auto number = [](const XmlRpc::XmlRpcValue & value) -> double {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
    throw std::invalid_argument("Planning acceleration map value is not numeric");
  };
  for (int i = 0; i < rows.size(); ++i) {
    if (rows[i].getType() != XmlRpc::XmlRpcValue::TypeArray || rows[i].size() != 2)
      throw std::invalid_argument("Planning acceleration map row must contain two numbers");
    map.emplace_back(number(rows[i][0]), number(rows[i][1]));
  }
  ROS_INFO_STREAM("Loaded shared planning propulsion envelope: " << map.size() << " knots");
  return AccelerationLimits{std::move(map)};
}
}  // namespace autoware::motion_utils
