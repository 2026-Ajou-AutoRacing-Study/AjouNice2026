#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_BOUNDARY_CORE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_BOUNDARY_CORE_HPP_

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/LineString.h>

#include <optional>

namespace autoware::behavior_path_planner
{

// Source GoalSearcher's minimum-boundary-margin pose calculation. The signed
// boundary intersection remains owned by the staged boundary-departure provider.
std::optional<geometry_msgs::Pose> calculateGoalPoseWithMinBoundaryMargin(
  const geometry_msgs::Pose & front_pose, const geometry_msgs::Pose & rear_pose,
  const lanelet::ConstLineString3d & boundary, double base_link2front,
  double margin_from_boundary, double vehicle_half_width, bool left_side_parking);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_BOUNDARY_CORE_HPP_
