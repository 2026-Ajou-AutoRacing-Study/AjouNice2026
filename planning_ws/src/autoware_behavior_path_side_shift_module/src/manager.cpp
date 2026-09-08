#include <autoware/behavior_path_side_shift_module/ros1_logging.hpp>
// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_path_side_shift_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

bool SideShiftModuleManager::init(ros::NodeHandle * node)
{
  // init manager interface
  if (!node || !SceneModuleManagerInterface::init(node, {})) return false;

  SideShiftParameters p{};

  const std::string ns = "side_shift.";
  p.min_distance_to_start_shifting =
    autoware_utils_rclcpp::detail::strict_get<double>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "min_distance_to_start_shifting"));
  p.time_to_start_shifting = autoware_utils_rclcpp::detail::strict_get<double>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "time_to_start_shifting"));
  p.shifting_lateral_jerk = autoware_utils_rclcpp::detail::strict_get<double>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "shifting_lateral_jerk"));
  p.min_shifting_distance = autoware_utils_rclcpp::detail::strict_get<double>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "min_shifting_distance"));
  p.min_shifting_speed = autoware_utils_rclcpp::detail::strict_get<double>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "min_shifting_speed"));
  p.shift_request_time_limit = autoware_utils_rclcpp::detail::strict_get<double>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "shift_request_time_limit"));
  p.publish_debug_marker = autoware_utils_rclcpp::detail::strict_get<bool>(*node,autoware_utils_rclcpp::to_ros1_parameter_name(ns + "publish_debug_marker"));

  parameters_ = std::make_shared<SideShiftParameters>(p);
  return true;
}

void SideShiftModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  [[maybe_unused]] auto p = parameters_;

  [[maybe_unused]] const std::string ns = "side_shift.";
  // update_param<bool>(parameters, ns + ..., ...);

  for(const auto & module:activeModules())
    std::dynamic_pointer_cast<SideShiftModule>(module)->updateModuleParams(p);
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SideShiftModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
