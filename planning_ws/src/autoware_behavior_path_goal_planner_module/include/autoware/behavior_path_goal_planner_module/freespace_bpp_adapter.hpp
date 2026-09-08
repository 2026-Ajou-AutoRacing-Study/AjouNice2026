#pragma once

#include <autoware/behavior_path_goal_planner_module/freespace_pull_over_core.hpp>
#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <optional>

namespace autoware::behavior_path_planner
{
// BPP-data adapter for the bounded freespace core. It is deliberately usable
// only for an observed empty predicted-object message; nonempty objects remain
// fail-closed until the source GoalSearcher/collision pipeline is ported.
class FreespaceBppAdapter
{
public:
  FreespaceBppAdapter(
    const ros::NodeHandle & node,
    const autoware::freespace_planning_algorithms::VehicleShape & vehicle_shape,
    double velocity, double acceleration = 0.0);

  std::optional<BehaviorModuleOutput> plan(
    const std::shared_ptr<const PlannerData> & planner_data,
    const geometry_msgs::Pose & end_pose, const geometry_msgs::Pose & goal_pose,
    size_t goal_id, bool use_back = false);

private:
  FreespacePullOverCore core_;
  double velocity_{};
  double acceleration_{};
};
}  // namespace autoware::behavior_path_planner
