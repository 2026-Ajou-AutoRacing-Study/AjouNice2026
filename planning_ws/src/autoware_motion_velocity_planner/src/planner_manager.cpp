#include "full_native_support.hpp"
// Copyright 2024 Tier IV, Inc.
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

#include "planner_manager.hpp"

#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils_system/stop_watch.hpp>

#include <boost/format.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

FullSourceMotionVelocityPlannerManager::FullSourceMotionVelocityPlannerManager()
: plugin_loader_(
    "autoware_motion_velocity_planner_common", "autoware::motion_velocity_planner::PluginModuleInterface")
{
}

void FullSourceMotionVelocityPlannerManager::load_module_plugin(ros::NodeHandle & node, const std::string & name)
{
  // Check if the plugin is already loaded.
  if (has_module(name)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger(node.getNamespace()), "The plugin '" << name << "' is already loaded.");
    return;
  }
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = std::shared_ptr<PluginModuleInterface>(plugin_loader_.createUniqueInstance(name));
    plugin->init(node, name);
    debug_trajectory_publishers_[name]=node.advertise<autoware_planning_msgs::Trajectory>("debug/"+plugin->getShortModuleName()+"/trajectory",1);

    // register
    loaded_plugins_.push_back(plugin);
    RCLCPP_DEBUG_STREAM(node.getNamespace(), "The scene plugin '" << name << "' is loaded.");

    // update the subscription
    const auto required_subscriptions = plugin->getRequiredSubscriptions();
    required_subscriptions_.update(required_subscriptions);
  } else {
    RCLCPP_ERROR_STREAM(node.getNamespace(), "The scene plugin '" << name << "' is not available.");
  }
}

void FullSourceMotionVelocityPlannerManager::unload_module_plugin(
  ros::NodeHandle & node, const std::string & name)
{
  auto it = std::remove_if(loaded_plugins_.begin(), loaded_plugins_.end(), [&](const auto plugin) {
    return plugin->getModuleName() == name;
  });

  if (it == loaded_plugins_.end()) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(node.getNamespace()),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    loaded_plugins_.erase(it, loaded_plugins_.end());
    debug_trajectory_publishers_.erase(name);
    required_subscriptions_={};
    for(const auto & plugin:loaded_plugins_) required_subscriptions_.update(plugin->getRequiredSubscriptions());
    RCLCPP_INFO_STREAM(node.getNamespace(), "The scene plugin '" << name << "' is unloaded.");
  }
}

void FullSourceMotionVelocityPlannerManager::update_module_parameters(
  const ParameterMap & parameters)
{
  for (auto & plugin : loaded_plugins_) plugin->updateParameters(parameters);
}

std::vector<VelocityPlanningResult> FullSourceMotionVelocityPlannerManager::plan_velocities(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data,
  std::shared_ptr<autoware_utils_debug::DebugPublisher> & processing_time_publisher)
{
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  std::vector<VelocityPlanningResult> results;
  for (auto & plugin : loaded_plugins_) {
    const auto plugin_name = plugin->getShortModuleName();
    stop_watch.tic(plugin_name);

    VelocityPlanningResult res =
      plugin->plan(raw_trajectory_points, smoothed_trajectory_points, planner_data);
    results.push_back(res);

    plugin->publishPlanningFactor();
    auto debug_trajectory = raw_trajectory_points;
    utils::applyPlanningResult(debug_trajectory, res);
    auto & publisher=debug_trajectory_publishers_.at(plugin->getModuleName());
    if(publisher.getNumSubscribers()>0) {
      autoware_planning_msgs::Trajectory message; message.header.frame_id="map";
      message.points=debug_trajectory; publisher.publish(message);
    }
    processing_time_publisher->publish<autoware_internal_debug_msgs::Float64Stamped>(
      std::string(plugin_name) + "/processing_time_ms", stop_watch.toc(plugin_name));
  }
  return results;
}
}  // namespace autoware::motion_velocity_planner
