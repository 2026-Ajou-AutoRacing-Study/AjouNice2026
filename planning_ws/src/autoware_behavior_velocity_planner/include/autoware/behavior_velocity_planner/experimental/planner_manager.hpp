#pragma once

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>

#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class BehaviorVelocityPlannerManager
{
public:
  BehaviorVelocityPlannerManager();
  [[nodiscard]] bool launchScenePlugin(ros::NodeHandle & node, const std::string & name);
  void removeScenePlugin(ros::NodeHandle & node, const std::string & name);
  Trajectory planPathVelocity(
    const PlannerData & planner_data, const Trajectory & input_path,
    const std_msgs::Header & header, const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound);
  RequiredSubscriptionInfo getRequiredSubscriptions() const { return required_subscriptions_; }

bool isPluginAvailable(const std::string & name) {
    return common_plugin_loader_.isClassAvailable(name) || plugin_loader_.isClassAvailable(name);
  }
  bool hasModule(const std::string & name) const {
    for (const auto & plugin : scene_manager_plugins_) if (plugin->getModuleName()==name) return true;
    return false;
  }
private:
  pluginlib::ClassLoader<PluginInterface> plugin_loader_;
  pluginlib::ClassLoader<PluginInterface> common_plugin_loader_;
  std::vector<std::shared_ptr<PluginInterface>> scene_manager_plugins_;
  RequiredSubscriptionInfo required_subscriptions_;
};
}  // namespace autoware::behavior_velocity_planner::experimental
