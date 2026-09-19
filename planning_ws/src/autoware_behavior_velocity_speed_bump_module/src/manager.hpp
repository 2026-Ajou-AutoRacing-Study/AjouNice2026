#pragma once
#include "scene.hpp"
#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <functional>
#include <memory>
namespace autoware::behavior_velocity_planner::experimental
{
class SpeedBumpModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit SpeedBumpModuleManager(ros::NodeHandle & node);
  const char * getModuleName() override { return "speed_bump"; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }
private:
  SpeedBumpModule::PlannerParam param_{};
  void launchNewModules(const Trajectory & path, const ros::Time & stamp, const PlannerData & data) override;
  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & data) override;
};
class SpeedBumpModulePlugin : public PluginWrapper<SpeedBumpModuleManager> {};
}  // namespace autoware::behavior_velocity_planner::experimental
