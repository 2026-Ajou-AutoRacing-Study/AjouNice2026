#include "ros1_compat.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace autoware::motion_velocity_planner::utils
{
std::string ros1_parameter_name(const std::string & name)
{
  auto result = name;
  std::replace(result.begin(), result.end(), '.', '/');
  return result;
}

std::vector<uint8_t> get_target_object_type(ros::NodeHandle & node, const std::string & param_prefix)
{
  const std::unordered_map<std::string, uint8_t> labels{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK}, {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};
  std::vector<uint8_t> enabled;
  for (const auto & label : labels) {
    bool selected = false;
    node.param(ros1_parameter_name(param_prefix + label.first), selected, false);
    if (selected) enabled.push_back(label.second);
  }
  return enabled;
}






}  // namespace autoware::motion_velocity_planner::utils
