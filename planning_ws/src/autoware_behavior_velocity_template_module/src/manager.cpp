#include "manager.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace autoware::behavior_velocity_planner
{
TemplateModuleManager::TemplateModuleManager(ros::NodeHandle & node)
: SceneModuleManagerInterface(node, getModuleName())
{
}

void TemplateModuleManager::launchNewModules(
  [[maybe_unused]] const experimental::Trajectory & path, [[maybe_unused]] const ros::Time & stamp,
  const PlannerData & planner_data)
{
  constexpr lanelet::Id module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(std::make_shared<TemplateModule>(
      module_id, node_, time_keeper_, planning_factor_interface_), planner_data);
  }
}

std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
TemplateModuleManager::getModuleExpiredFunction(
  [[maybe_unused]] const experimental::Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return [](const std::shared_ptr<experimental::SceneModuleInterface> &) { return false; };
}
}  // namespace autoware::behavior_velocity_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::TemplateModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
