#pragma once

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>

namespace autoware::behavior_velocity_planner::experimental
{
// The frozen ROS2 default launch selects the experimental ABI.  Reuse the
// already-portered ROS1 stop-line rule through an explicit ABI adapter: both
// paths operate on the same PathWithLaneId representation and parameters.
class StopLineModulePlugin final : public PluginInterface
{
public:
  void init(ros::NodeHandle & node) override;
  RequiredSubscriptionInfo getRequiredSubscriptions() override;
  void plan(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound,
    const PlannerData & planner_data) override;
  void updateSceneModuleInstances(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override;
  const char * getModuleName() override;

private:
  autoware::behavior_velocity_planner::StopLineModulePlugin legacy_plugin_;
};
}  // namespace autoware::behavior_velocity_planner::experimental
