#pragma once

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

namespace autoware::behavior_velocity_planner
{
class TemplateModuleManager : public experimental::SceneModuleManagerInterface<>
{
public:
  explicit TemplateModuleManager(ros::NodeHandle & node);
  const char * getModuleName() override { return "template"; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }

private:
  void launchNewModules(
    const experimental::Trajectory & path, const ros::Time & stamp,
    const PlannerData & planner_data) override;
  std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
  getModuleExpiredFunction(
    const experimental::Trajectory & path, const PlannerData & planner_data) override;
};

class TemplateModulePlugin : public experimental::PluginWrapper<TemplateModuleManager> {};
}  // namespace autoware::behavior_velocity_planner
