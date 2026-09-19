#pragma once

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner::experimental
{
template <class T>
class PluginWrapper : public PluginInterface
{
public:
  void init(ros::NodeHandle & node) override { scene_manager_ = std::make_unique<T>(node); }
  RequiredSubscriptionInfo getRequiredSubscriptions() override
  {
    return scene_manager_->getRequiredSubscriptions();
  }
  void plan(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data) override
  {
    scene_manager_->plan(path, header, left_bound, right_bound, planner_data);
  }
  void updateSceneModuleInstances(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override
  {
    scene_manager_->updateSceneModuleInstances(path, stamp, planner_data);
  }
  const char * getModuleName() override { return scene_manager_->getModuleName(); }

private:
  std::unique_ptr<T> scene_manager_;
};
}  // namespace autoware::behavior_velocity_planner::experimental
