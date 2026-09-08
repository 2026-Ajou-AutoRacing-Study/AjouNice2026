#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_SEARCH_CORE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_SEARCH_CORE_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate_generator_core.hpp>

#include <lanelet2_core/primitives/LineString.h>

namespace autoware::behavior_path_planner
{
struct GoalCandidateSearchParameters
{
  bool left_side_parking{true};
  double base_link2front{0.0};
  double vehicle_length{0.0};
  double vehicle_width{0.0};
  double margin_from_boundary{0.0};
  double backward_goal_search_length{0.0};
  double forward_goal_search_length{0.0};
  double longitudinal_interval{0.0};
  double max_lateral_offset{0.0};
  double lateral_offset_interval{0.0};
};

// Runs the source search() boundary branch from a selected outer boundary. Map
// selection, regulatory-area extraction, and departure-lane construction stay
// explicit providers at this candidate boundary.
GoalCandidates searchGoalCandidatesAlongBoundary(
  const lanelet::ConstLineString3d & boundary, const geometry_msgs::Pose & reference_goal_pose,
  const GoalCandidateSearchParameters & parameters,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateAreaConstraints & area_constraints);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_SEARCH_CORE_HPP_
