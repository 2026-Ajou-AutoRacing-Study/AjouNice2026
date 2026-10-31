#include "autoware/behavior_velocity_planner/experimental/planner_manager.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>

namespace autoware::behavior_velocity_planner::experimental
{
namespace
{
ros::Publisher & velocity_trace_publisher()
{
  static ros::Publisher publisher;
  return publisher;
}
}
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
  if (!velocity_trace_publisher()) {
    velocity_trace_publisher() = node.advertise<std_msgs::String>("debug/module_velocity_changes", 1);
  }
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
  const bool observe = velocity_trace_publisher().getNumSubscribers() > 0;
  nlohmann::json trace{{"stamp", ros::Time::now().toSec()},
    {"input_stamp", header.stamp.toSec()}, {"modules", nlohmann::json::array()}};
  for (const auto & plugin : scene_manager_plugins_) {
    plugin->updateSceneModuleInstances(input_path, header.stamp, planner_data);
    const auto before = observe ? output_path.restore() : decltype(output_path.restore()){};
    plugin->plan(output_path, header, left_bound, right_bound, planner_data);
    if (observe) {
      const auto after = output_path.restore();
      nlohmann::json changes = nlohmann::json::array();
      double before_min = 1e9, after_min = 1e9;
      for (const auto & point : before) {
        before_min = std::min(before_min, double(point.point.longitudinal_velocity_mps));
      }
      size_t changed_count = 0;
      for (size_t i = 0; i < after.size(); ++i) {
        const auto & point = after[i].point;
        after_min = std::min(after_min, double(point.longitudinal_velocity_mps));
        if (before.empty()) continue;
        size_t match = std::min(i, before.size()-1);
        auto distance = [&](size_t j) {
          return std::hypot(before[j].point.pose.position.x-point.pose.position.x,
            before[j].point.pose.position.y-point.pose.position.y);
        };
        if (before.size() != after.size() || distance(match) > 1e-3) {
          for (size_t j = 0; j < before.size(); ++j) {
            if (distance(j) < distance(match)) match = j;
          }
        }
        const double previous = before[match].point.longitudinal_velocity_mps;
        if (point.longitudinal_velocity_mps < previous - 0.01) {
          ++changed_count;
          if (changes.size() < 64) changes.push_back({{"index", i},
            {"x", point.pose.position.x}, {"y", point.pose.position.y},
            {"before_mps", previous}, {"after_mps", point.longitudinal_velocity_mps},
            {"geometry_match_distance_m", distance(match)}});
        }
      }
      trace["modules"].push_back({{"name", plugin->getModuleName()},
        {"before_min_mps", before_min}, {"after_min_mps", after_min},
        {"changed_count", changed_count}, {"changes", changes}});
    }
  }
  if (observe) {
    std_msgs::String message; message.data = trace.dump();
    velocity_trace_publisher().publish(message);
  }
  return output_path;
}
}  // namespace autoware::behavior_velocity_planner::experimental
