#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_GENERATOR_CORE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_GENERATOR_CORE_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate.hpp>
#include <autoware/behavior_path_goal_planner_module/goal_candidate_area_filter.hpp>

#include <vector>

namespace autoware::behavior_path_planner
{
struct GoalCandidateSeed
{
  geometry_msgs::Pose pose_with_min_boundary_margin{};
  double distance_from_original_goal{0.0};
};

// Expands the source search() lateral loop after boundary sampling and
// minimum-margin calculation. A non-positive interval fails closed instead of
// entering the source configuration's otherwise unbounded loop.
GoalCandidates generateGoalCandidatesFromSeeds(
  const std::vector<GoalCandidateSeed> & seeds, bool left_side_parking,
  double max_lateral_offset, double lateral_offset_interval,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateAreaConstraints & area_constraints);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_GENERATOR_CORE_HPP_
