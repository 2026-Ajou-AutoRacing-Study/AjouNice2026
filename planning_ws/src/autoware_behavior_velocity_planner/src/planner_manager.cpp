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

#include "autoware/behavior_velocity_planner/planner_manager.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <boost/format.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::behavior_velocity_planner
{
BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
: plugin_loader_(
    // Preserve the frozen plugin contract: scene modules export their
    // descriptions against the planner package.
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::PluginInterface"),
  common_plugin_loader_(
    "autoware_behavior_velocity_planner_common",
    "autoware::behavior_velocity_planner::PluginInterface")
{
}

void BehaviorVelocityPlannerManager::launchScenePlugin(
  ros::NodeHandle & node, const std::string & name)
{
  auto & loader = common_plugin_loader_.isClassAvailable(name) ? common_plugin_loader_ : plugin_loader_;
  if (loader.isClassAvailable(name)) {
    // ROS1 pluginlib exposes a unique-instance factory rather than ROS2's
    // shared-instance API. Transfer that ownership once into the manager's
    // shared plugin lifetime.
    const auto plugin =
      std::shared_ptr<PluginInterface>(loader.createUniqueInstance(name));
    plugin->init(node);

    // Check if the plugin is already registered.
    for (const auto & running_plugin : scene_manager_plugins_) {
      if (plugin->getModuleName() == running_plugin->getModuleName()) {
        ROS_WARN_STREAM_NAMED(node.getNamespace(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // register
    scene_manager_plugins_.push_back(plugin);
    ROS_INFO_STREAM_NAMED(node.getNamespace(), "The scene plugin '" << name << "' is loaded.");

    // update the subscription
    const auto required_subscriptions = plugin->getRequiredSubscriptions();
    required_subscriptions_.concat(required_subscriptions);
  } else {
    ROS_ERROR_STREAM_NAMED(node.getNamespace(), "The scene plugin '" << name << "' is not available.");
  }
}

void BehaviorVelocityPlannerManager::removeScenePlugin(
  ros::NodeHandle & node, const std::string & name)
{
  auto it = std::remove_if(
    scene_manager_plugins_.begin(), scene_manager_plugins_.end(),
    [&](const std::shared_ptr<behavior_velocity_planner::PluginInterface> plugin) {
      return plugin->getModuleName() == name;
    });

  if (it == scene_manager_plugins_.end()) {
    ROS_WARN_STREAM_NAMED(
      node.getNamespace(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    scene_manager_plugins_.erase(it, scene_manager_plugins_.end());
    ROS_INFO_STREAM_NAMED(node.getNamespace(), "The scene plugin '" << name << "' is unloaded.");
  }
}

autoware_internal_planning_msgs::PathWithLaneId
BehaviorVelocityPlannerManager::planPathVelocity(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_internal_planning_msgs::PathWithLaneId & input_path_msg)
{
  autoware_internal_planning_msgs::PathWithLaneId output_path_msg = input_path_msg;

  for (const auto & plugin : scene_manager_plugins_) {
    plugin->updateSceneModuleInstances(planner_data, input_path_msg);
    plugin->plan(&output_path_msg);
  }

  return output_path_msg;
}

}  // namespace autoware::behavior_velocity_planner
