#pragma once

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_manager_interface.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class SpeedBumpModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit SpeedBumpModuleManager(ros::NodeHandle & node);
  const char * getModuleName() override { return "speed_bump"; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }

private:
  SpeedBumpModule::PlannerParam planner_param_{};
  void launchNewModules(const autoware_internal_planning_msgs::PathWithLaneId & path) override;
  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::PathWithLaneId & path) override;
};

class SpeedBumpModulePlugin : public PluginWrapper<SpeedBumpModuleManager> {};
}  // namespace autoware::behavior_velocity_planner
