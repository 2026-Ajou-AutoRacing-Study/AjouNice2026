// Copyright 2025 Tier IV, Inc.
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

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using lanelet::autoware::VirtualTrafficLight;

VirtualTrafficLightModuleManager::VirtualTrafficLightModuleManager(ros::NodeHandle & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(VirtualTrafficLightModuleManager::getModuleName());

  {
    auto & p = planner_param_;
    node.param(ns + "/max_delay_sec", p.max_delay_sec, 1.0);
    node.param(ns + "/near_line_distance", p.near_line_distance, 0.0);
    node.param(ns + "/dead_line_margin", p.dead_line_margin, 0.0);
    node.param(ns + "/hold_stop_margin_distance", p.hold_stop_margin_distance, 0.0);
    double max_yaw_deviation_deg = 0.0;
    node.param(ns + "/max_yaw_deviation_deg", max_yaw_deviation_deg, 0.0);
    p.max_yaw_deviation_rad = autoware_utils::deg2rad(max_yaw_deviation_deg);
    node.param(ns + "/check_timeout_after_stop_line", p.check_timeout_after_stop_line, false);
  }

  sub_virtual_traffic_light_states_ = node.subscribe(
    "input/virtual_traffic_light_states", 1,
    &VirtualTrafficLightModuleManager::onVirtualTrafficLightStates, this);
  pub_infrastructure_commands_ = node.advertise<tier4_v2x_msgs::InfrastructureCommandArray>(
    "output/infrastructure_commands", 1);
}

void VirtualTrafficLightModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const ros::Time & stamp,
  const PlannerData & planner_data)
{
  autoware_internal_planning_msgs::PathWithLaneId path_msg;
  path_msg.points = path.restore();

  autoware_utils::LineString2d ego_path_linestring;
  for (const auto & path_point : path_msg.points) {
    ego_path_linestring.push_back(autoware_utils::from_msg(path_point.point.pose.position).to_2d());
  }

  for (const auto & m : planning_utils::getRegElemMapOnPath<VirtualTrafficLight>(
         path_msg, planner_data.route_handler_->getLaneletMapPtr(),
         planner_data.current_odometry->pose)) {
    const auto stop_line_opt = m.first->getStopLine();
    if (!stop_line_opt) {
      ROS_FATAL(
        "No stop line at virtual_traffic_light_reg_elem_id = %ld, please fix the map!",
        m.first->id());
      continue;
    }

    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = m.second.id();
    const auto module_id = lane_id;
    if (
      !isModuleRegistered(module_id) &&
      boost::geometry::intersects(
        ego_path_linestring, lanelet::utils::to2D(stop_line_opt.value()).basicLineString())) {
      registerModule(
        std::make_shared<VirtualTrafficLightModule>(
          module_id, lane_id, *m.first, m.second, planner_param_,
          node_, time_keeper_,
          planning_factor_interface_),
        planner_data);
    }
  }
}

std::function<bool(const std::shared_ptr<VirtualTrafficLightModule> &)>
VirtualTrafficLightModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  autoware_internal_planning_msgs::PathWithLaneId path_msg;
  path_msg.points = path.restore();

  const auto id_set = planning_utils::getLaneIdSetOnPath(
    path_msg, planner_data.route_handler_->getLaneletMapPtr(), planner_data.current_odometry->pose);

  return [id_set](const std::shared_ptr<VirtualTrafficLightModule> & scene_module) {
    return id_set.count(scene_module->getModuleId()) == 0;
  };
}

void VirtualTrafficLightModuleManager::plan(
  Trajectory & path, const std_msgs::Header & header,
  const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data)
{
  // NOTE: virtual traffic light specific implementation
  //       Since the argument of modifyPathVelocity cannot be changed, the specific information
  //       of virtual traffic light states is set here.
  for (const auto & scene_module : scene_modules_) {
    scene_module->setCorrespondingVirtualTrafficLightState(virtual_traffic_light_states_);
  }

  SceneModuleManagerInterface<VirtualTrafficLightModule>::plan(
    path, header, left_bound, right_bound, planner_data);

  // NOTE: virtual traffic light specific implementation
  //       publish infrastructure_command_array
  tier4_v2x_msgs::InfrastructureCommandArray infrastructure_command_array;
  infrastructure_command_array.stamp.sec = header.stamp.sec;
  infrastructure_command_array.stamp.nanosec = header.stamp.nsec;

  for (const auto & scene_module : scene_modules_) {
    if (const auto command = scene_module->getInfrastructureCommand()) {
      infrastructure_command_array.commands.push_back(*command);
    }
  }
  pub_infrastructure_commands_.publish(infrastructure_command_array);
}
}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::VirtualTrafficLightModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
