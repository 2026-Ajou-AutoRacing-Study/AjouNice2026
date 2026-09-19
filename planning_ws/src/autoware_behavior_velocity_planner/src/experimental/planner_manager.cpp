#include "autoware/behavior_velocity_planner/experimental/planner_manager.hpp"

#include <algorithm>

namespace autoware::behavior_velocity_planner::experimental
{
BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
: plugin_loader_(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface"),
  common_plugin_loader_(
    "autoware_behavior_velocity_planner_common",
    "autoware::behavior_velocity_planner::experimental::PluginInterface")
{
}

bool BehaviorVelocityPlannerManager::launchScenePlugin(
  ros::NodeHandle & node, const std::string & name)
{
  auto & loader = common_plugin_loader_.isClassAvailable(name) ? common_plugin_loader_ : plugin_loader_;
  if (!loader.isClassAvailable(name)) {
    ROS_ERROR_STREAM_NAMED(node.getNamespace(), "The experimental scene plugin '" << name << "' is not available.");
    return false;
  }
  const auto plugin = std::shared_ptr<PluginInterface>(loader.createUniqueInstance(name));
  plugin->init(node);
  for (const auto & running_plugin : scene_manager_plugins_) {
    if (plugin->getModuleName() == running_plugin->getModuleName()) {
    ROS_WARN_STREAM_NAMED(node.getNamespace(), "The experimental scene plugin '" << name << "' is already loaded.");
    return false;
    }
  }
  required_subscriptions_.concat(plugin->getRequiredSubscriptions());
  scene_manager_plugins_.push_back(plugin);
  ROS_INFO_STREAM_NAMED(node.getNamespace(), "The experimental scene plugin '" << name << "' is loaded.");
  return true;
}

void BehaviorVelocityPlannerManager::removeScenePlugin(
  ros::NodeHandle & node, const std::string & name)
{
  const auto it = std::remove_if(
    scene_manager_plugins_.begin(), scene_manager_plugins_.end(),
    [&](const std::shared_ptr<PluginInterface> & plugin) { return plugin->getModuleName() == name; });
  if (it == scene_manager_plugins_.end()) {
    ROS_WARN_STREAM_NAMED(node.getNamespace(), "The experimental scene plugin '" << name << "' is not found.");
    return;
  }
  scene_manager_plugins_.erase(it, scene_manager_plugins_.end());
}

Trajectory BehaviorVelocityPlannerManager::planPathVelocity(
  const PlannerData & planner_data, const Trajectory & input_path,
  const std_msgs::Header & header, const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound)
{
  auto output_path = input_path;
  for (const auto & plugin : scene_manager_plugins_) {
    plugin->updateSceneModuleInstances(input_path, header.stamp, planner_data);
    plugin->plan(output_path, header, left_bound, right_bound, planner_data);
  }
  return output_path;
}
}  // namespace autoware::behavior_velocity_planner::experimental
