#pragma once

#include <autoware/motion_velocity_planner/motion_velocity_planner_manager_core.hpp>
#include <pluginlib/class_loader.hpp>

namespace autoware::motion_velocity_planner
{
// Pluginlib boundary matching the upstream manager. It intentionally refuses
// unavailable classes instead of treating requested launch modules as empty.
class MotionVelocityPlannerManager
{
public:
  MotionVelocityPlannerManager();
  bool launchModule(ros::NodeHandle & node, const std::string & name);
  std::vector<VelocityPlanningResult> planVelocities(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data)
  {
    return core_.planVelocities(raw_trajectory_points, smoothed_trajectory_points, planner_data);
  }
  const RequiredSubscriptionInfo & requiredSubscriptions() const { return core_.requiredSubscriptions(); }
  size_t size() const { return core_.size(); }

private:
  pluginlib::ClassLoader<PluginModuleInterface> plugin_loader_;
  MotionVelocityPlannerManagerCore core_;
};
}  // namespace autoware::motion_velocity_planner
