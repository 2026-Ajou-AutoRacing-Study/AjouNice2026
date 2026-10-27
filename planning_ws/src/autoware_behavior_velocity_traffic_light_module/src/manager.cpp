// Copyright 2020 Tier IV, Inc.
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
#include <autoware/behavior_velocity_traffic_light_module/signal_judging_line_ros.hpp>
#include "utils.hpp"

#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <tf2/utils.h>

#include <limits>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <set>
#include <string>
#include <utility>
namespace autoware::behavior_velocity_planner
{
using traffic_light_ros1::require_parameter;
using lanelet::TrafficLight;

TrafficLightModuleManager::TrafficLightModuleManager(ros::NodeHandle & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(), getEnableRTC(node, std::string(getModuleName()) + "/enable_rtc"))
{
  const std::string ns(TrafficLightModuleManager::getModuleName());
  planner_param_.stop_margin = require_parameter<double>(node, ns + ".stop_margin");
  planner_param_.tl_state_timeout =
    require_parameter<double>(node, ns + ".tl_state_timeout");
  planner_param_.stop_time_hysteresis =
    require_parameter<double>(node, ns + ".stop_time_hysteresis");
  planner_param_.enable_pass_judge =
    require_parameter<bool>(node, ns + ".enable_pass_judge");
  node.param<bool>(ns + "/enable_yellow_passthrough", planner_param_.enable_yellow_passthrough, false);
  node.param<bool>(ns + "/yellow_light_is_go", planner_param_.yellow_light_is_go, false);
  node.param<bool>(ns + "/competition_fail_open", planner_param_.competition_fail_open, false);
  node.param<bool>(ns + "/enable_signal_compliance_guard", planner_param_.enable_signal_compliance_guard, false);
  node.param<bool>(ns + "/defer_unseen_until_observed", planner_param_.defer_unseen_until_observed, false);
  node.param<bool>(ns + "/defer_far_unseen_unknown", planner_param_.defer_far_unseen_unknown, false);
  node.param<double>(ns + "/far_unseen_min_stop_distance_m", planner_param_.far_unseen_min_stop_distance_m, 40.0);
  node.param<double>(ns + "/far_unseen_response_margin_sec", planner_param_.far_unseen_response_margin_sec, 1.0);
  node.param<double>(ns + "/signal_compliance_front_axle_offset_m", planner_param_.signal_compliance_front_axle_offset_m, -1.0);
  planner_param_.use_official_judging_lines = node.hasParam(ns + "/signal_judging_lines");
  planner_param_.signal_judging_lines = signal_compliance::loadSignalJudgingLines(node, ns + "/signal_judging_lines");
  node.param<int>(ns + "/signal_compliance_required_green_observations", planner_param_.signal_compliance_required_green_observations, 3);
  node.param<double>(ns + "/signal_compliance_unknown_grace_sec", planner_param_.signal_compliance_unknown_grace_sec, 0.0);
  node.param<double>(ns + "/signal_compliance_yellow_deceleration_mps2", planner_param_.signal_compliance_yellow_deceleration_mps2, 6.0);
  node.param<double>(ns + "/signal_compliance_yellow_jerk_mps3", planner_param_.signal_compliance_yellow_jerk_mps3, 10.0);
  // Read obsolete timed-yellow values for launch/snapshot compatibility only.
  // They cannot select or veto the new Lanelet stopping-distance policy.
  node.param<bool>(ns + "/signal_compliance_enable_timed_yellow", planner_param_.signal_compliance_enable_timed_yellow, false);
  node.param<double>(ns + "/signal_compliance_min_yellow_sec", planner_param_.signal_compliance_min_yellow_sec, 2.0);
  node.param<double>(ns + "/signal_compliance_yellow_time_margin_sec", planner_param_.signal_compliance_yellow_time_margin_sec, 0.2);
  if (planner_param_.signal_compliance_required_green_observations < 1 ||
      planner_param_.signal_compliance_required_green_observations > 3 ||
      !std::isfinite(planner_param_.signal_compliance_unknown_grace_sec) ||
      planner_param_.signal_compliance_unknown_grace_sec < 0.0 ||
      planner_param_.signal_compliance_unknown_grace_sec > 0.3 ||
      (planner_param_.signal_compliance_unknown_grace_sec > 0.0 && !planner_param_.enable_signal_compliance_guard) ||
      !std::isfinite(planner_param_.signal_compliance_yellow_deceleration_mps2) ||
      planner_param_.signal_compliance_yellow_deceleration_mps2 <= 0.0 ||
      !std::isfinite(planner_param_.signal_compliance_yellow_jerk_mps3) ||
      planner_param_.signal_compliance_yellow_jerk_mps3 <= 0.0) {
    throw std::invalid_argument("invalid signal confirmation/grace or yellow braking parameters");
  }
  if (planner_param_.enable_signal_compliance_guard &&
      (!std::isfinite(planner_param_.signal_compliance_front_axle_offset_m) ||
       planner_param_.signal_compliance_front_axle_offset_m < 0.0)) {
    throw std::invalid_argument("signal compliance candidate requires an explicitly qualified front axle offset");
  }
  if (planner_param_.defer_far_unseen_unknown &&
      (!planner_param_.enable_signal_compliance_guard ||
       !std::isfinite(planner_param_.far_unseen_min_stop_distance_m) ||
       planner_param_.far_unseen_min_stop_distance_m <= 0.0 ||
       !std::isfinite(planner_param_.far_unseen_response_margin_sec) ||
       planner_param_.far_unseen_response_margin_sec < 0.0)) {
    throw std::invalid_argument("invalid far-unseen traffic signal parameters");
  }
  if (planner_param_.defer_unseen_until_observed && !planner_param_.enable_signal_compliance_guard) {
    throw std::invalid_argument("initial-unseen traffic signal policy requires compliance guard");
  }
  planner_param_.yellow_lamp_period =
    require_parameter<double>(node, ns + ".yellow_lamp_period");
  planner_param_.yellow_light_stop_velocity =
    require_parameter<double>(node, ns + ".yellow_light_stop_velocity");
  planner_param_.enable_arrow_aware_yellow_passing =
    require_parameter<bool>(node, ns + ".enable_arrow_aware_yellow_passing");
  planner_param_.min_behind_dist_to_stop_for_restart_suppression =
    require_parameter<double>(node, ns + ".restart_suppression.min_behind_distance_to_stop");
  planner_param_.max_behind_dist_to_stop_for_restart_suppression =
    require_parameter<double>(node, ns + ".restart_suppression.max_behind_distance_to_stop");
  planner_param_.v2i_use_remaining_time =
    require_parameter<bool>(node, ns + ".v2i.use_remaining_time");
  planner_param_.v2i_last_time_allowed_to_pass =
    require_parameter<double>(node, ns + ".v2i.last_time_allowed_to_pass");
  planner_param_.v2i_velocity_threshold =
    require_parameter<double>(node, ns + ".v2i.velocity_threshold");
  planner_param_.v2i_required_time_to_departure =
    require_parameter<double>(node, ns + ".v2i.required_time_to_departure");
  std::vector<int> ignored_regulatory_ids;
  node.getParam(ns + "/ignored_regulatory_ids", ignored_regulatory_ids);
  ignored_regulatory_ids_.insert(ignored_regulatory_ids.begin(), ignored_regulatory_ids.end());
  pub_tl_state_ = node.advertise<autoware_perception_msgs::TrafficLightGroup>(
    "output/traffic_signal", 1);
}

void TrafficLightModuleManager::modifyPathVelocity(
  autoware_internal_planning_msgs::PathWithLaneId * path)
{
  visualization_msgs::MarkerArray debug_marker_array;
  visualization_msgs::MarkerArray virtual_wall_marker_array;

  autoware_perception_msgs::TrafficLightGroup tl_state;

  nearest_ref_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  for (const auto & scene_module : scene_modules_) {
    std::shared_ptr<TrafficLightModule> traffic_light_scene_module(
      std::dynamic_pointer_cast<TrafficLightModule>(scene_module));
    traffic_light_scene_module->setPlannerData(planner_data_);
    traffic_light_scene_module->modifyPathVelocity(path);

    if (
      traffic_light_scene_module->getFirstRefStopPathPointIndex() <
      nearest_ref_stop_path_point_index_) {
      nearest_ref_stop_path_point_index_ =
        traffic_light_scene_module->getFirstRefStopPathPointIndex();
      if (
        traffic_light_scene_module->getTrafficLightModuleState() !=
        TrafficLightModule::State::GO_OUT) {
        tl_state = traffic_light_scene_module->getTrafficSignal();
      }
    }
    for (const auto & marker : traffic_light_scene_module->createDebugMarkerArray().markers) {
      debug_marker_array.markers.push_back(marker);
    }
    virtual_wall_marker_creator_.add_virtual_walls(
      traffic_light_scene_module->createVirtualWalls());
  }
  planning_factor_interface_->publish();
  pub_debug_.publish(debug_marker_array);
  pub_virtual_wall_.publish(virtual_wall_marker_creator_.create_markers(clock_->now()));
  pub_tl_state_.publish(tl_state);
}

void TrafficLightModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  for (const auto & traffic_light_reg_elem :
       planning_utils::getRegElemMapOnPath<lanelet::autoware::AutowareTrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    if (ignored_regulatory_ids_.count(traffic_light_reg_elem.first->id()) != 0U) {
      continue;
    }
    const auto stop_line = traffic_light_reg_elem.first->stopLine();

    if (!stop_line) {
      RCLCPP_FATAL(
        logger_, "No stop line at traffic_light_reg_elem_id = %ld, please fix the map!",
        traffic_light_reg_elem.first->id());
      continue;
    }

    const auto & lane = traffic_light_reg_elem.second;
    lanelet::ConstLanelets following_lanes;
    const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
    if (routing_graph) {
      following_lanes = routing_graph->following(lane);
    }
    const auto decision_lane = selectTrafficSignalDecisionLane(lane, path, following_lanes);
    const bool is_turn_lane =
      autoware::experimental::lanelet2_utils::is_left_direction(decision_lane) ||
      autoware::experimental::lanelet2_utils::is_right_direction(decision_lane);

    const bool has_static_arrow = hasStaticArrow(traffic_light_reg_elem.first);

    const auto lane_id = traffic_light_reg_elem.second.id();
    auto existing_module = this->getRegisteredAssociatedModule(lane_id);
    if (!existing_module) {
      registerModule(
        std::make_shared<TrafficLightModule>(
          lane_id, *(traffic_light_reg_elem.first), decision_lane, *stop_line,
          is_turn_lane, has_static_arrow, planner_param_, *node_, (logger_ + ".traffic_light_module"),
          clock_, time_keeper_, planning_factor_interface_));
      generateUUID(lane_id);
      updateRTCStatus(
        getUUID(lane_id), true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
        path.header.stamp);
    } else {
      // Update the stop line for the existing module
      existing_module->updateStopLine(*stop_line);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
TrafficLightModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  const auto lanelet_id_set = planning_utils::getLaneletIdSetOnPath<TrafficLight>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [this, lanelet_id_set](
           [[maybe_unused]] const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
    for (const auto & id : lanelet_id_set) {
      if (this->getRegisteredAssociatedModule(id)) {
        return false;
      }
    }
    return true;
  };
}

std::shared_ptr<TrafficLightModule> TrafficLightModuleManager::getRegisteredAssociatedModule(
  const lanelet::Id & id) const
{
  const auto lane = planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(id);

  for (const auto & registered_id : registered_module_id_set_) {
    if (hasAssociatedTrafficLight(lane, registered_id)) {
      return findModuleById(registered_id);
    }
  }
  return nullptr;
}

bool TrafficLightModuleManager::hasAssociatedTrafficLight(
  const lanelet::ConstLanelet & lane, const lanelet::Id & registered_id) const
{
  const auto registered_lane =
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(registered_id);

  for (const auto & registered_element : registered_lane.regulatoryElementsAs<TrafficLight>()) {
    for (const auto & element : lane.regulatoryElementsAs<TrafficLight>()) {
      if (hasSameTrafficLight(element, registered_element)) {
        return true;
      }
    }
  }
  return false;
}

std::shared_ptr<TrafficLightModule> TrafficLightModuleManager::findModuleById(
  const lanelet::Id & module_id) const
{
  for (const auto & scene_module : scene_modules_) {
    if (scene_module->getModuleId() == module_id) {
      return std::dynamic_pointer_cast<TrafficLightModule>(scene_module);
    }
  }
  return nullptr;
}

bool TrafficLightModuleManager::hasSameTrafficLight(
  const lanelet::TrafficLightConstPtr element,
  const lanelet::TrafficLightConstPtr registered_element) const
{
  for (const auto & traffic_light : element->trafficLights()) {
    for (const auto & registered_traffic_light : registered_element->trafficLights()) {
      if (traffic_light.id() == registered_traffic_light.id()) {
        return true;
      }
    }
  }
  return false;
}

bool TrafficLightModuleManager::hasStaticArrow(
  const std::shared_ptr<const lanelet::autoware::AutowareTrafficLight> & reg_elem) const
{
  for (const auto & light : reg_elem->trafficLights()) {
    const auto & attributes = light.attributes();
    if (attributes.find("subtype") != attributes.end()) {
      const std::string subtype = attributes.at("subtype").value();
      if (subtype.find("arrow") != std::string::npos) {
        return true;
      }
    }
  }

  for (const auto & light_bulb_ls : reg_elem->lightBulbs()) {
    for (const auto & node : light_bulb_ls) {
      const auto & attributes = node.attributes();
      if (attributes.find("arrow") != attributes.end()) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::TrafficLightModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
