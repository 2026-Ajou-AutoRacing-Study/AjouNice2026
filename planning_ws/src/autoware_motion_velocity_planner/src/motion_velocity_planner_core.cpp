#include <autoware/motion_velocity_planner/motion_velocity_planner_core.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>

namespace autoware::motion_velocity_planner
{
std::vector<autoware_planning_msgs::TrajectoryPoint> MotionVelocityPlannerCore::applyResults(
  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory,
  const std::vector<VelocityPlanningResult> & results) const
{
  for (const auto & result : results) utils::applyPlanningResult(trajectory, result);
  return trajectory;
}
}  // namespace autoware::motion_velocity_planner
