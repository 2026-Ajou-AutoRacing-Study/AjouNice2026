#include <autoware/motion_velocity_planner/motion_velocity_planner_manager_core.hpp>

#include <algorithm>

namespace autoware::motion_velocity_planner
{
bool MotionVelocityPlannerManagerCore::registerModule(
  ros::NodeHandle & node, const std::string & module_name,
  const std::shared_ptr<PluginModuleInterface> & module)
{
  if (!module || module_name.empty()) return false;
  const auto duplicate = std::any_of(modules_.begin(), modules_.end(), [&](const auto & existing) {
    return existing->getModuleName() == module_name;
  });
  if (duplicate) return false;
  module->init(node, module_name);
  required_subscriptions_.update(module->getRequiredSubscriptions());
  modules_.push_back(module);
  return true;
}

std::vector<VelocityPlanningResult> MotionVelocityPlannerManagerCore::planVelocities(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  std::vector<VelocityPlanningResult> results;
  results.reserve(modules_.size());
  for (const auto & module : modules_) {
    results.push_back(module->plan(raw_trajectory_points, smoothed_trajectory_points, planner_data));
    module->publishPlanningFactor();
  }
  return results;
}
}  // namespace autoware::motion_velocity_planner
