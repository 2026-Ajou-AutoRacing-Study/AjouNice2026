#pragma once

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
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

  void plan(autoware_internal_planning_msgs::PathWithLaneId * path) override
  {
    scene_manager_->plan(path);
  }

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_internal_planning_msgs::PathWithLaneId & path) override
  {
    scene_manager_->updateSceneModuleInstances(planner_data, path);
  }

  const char * getModuleName() override { return scene_manager_->getModuleName(); }

private:
  std::unique_ptr<T> scene_manager_;
};

}  // namespace autoware::behavior_velocity_planner
