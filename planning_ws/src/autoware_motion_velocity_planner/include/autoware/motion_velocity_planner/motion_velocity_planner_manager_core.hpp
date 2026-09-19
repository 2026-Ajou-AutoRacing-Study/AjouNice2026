#pragma once

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
// ROS1-safe deterministic subset of the upstream manager. Plugin discovery,
// PlannerData transport and debug publication intentionally remain node work.
class MotionVelocityPlannerManagerCore
{
public:
  bool registerModule(
    ros::NodeHandle & node, const std::string & module_name,
    const std::shared_ptr<PluginModuleInterface> & module);

  std::vector<VelocityPlanningResult> planVelocities(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data);

  const RequiredSubscriptionInfo & requiredSubscriptions() const { return required_subscriptions_; }
  size_t size() const { return modules_.size(); }

private:
  std::vector<std::shared_ptr<PluginModuleInterface>> modules_;
  RequiredSubscriptionInfo required_subscriptions_;
};
}  // namespace autoware::motion_velocity_planner
