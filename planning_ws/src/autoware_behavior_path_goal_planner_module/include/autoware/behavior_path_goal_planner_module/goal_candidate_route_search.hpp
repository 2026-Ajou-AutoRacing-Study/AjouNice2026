#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_ROUTE_SEARCH_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_ROUTE_SEARCH_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate_search_core.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <memory>

namespace autoware::behavior_path_planner
{

// RouteHandler integration for the source GoalSearcher boundary selection.
// Regulatory-area extraction is intentionally represented by area_constraints.
GoalCandidates searchGoalCandidatesFromRoute(
  const std::shared_ptr<const autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & reference_goal_pose,
  const GoalCandidateSearchParameters & parameters,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateAreaConstraints & area_constraints);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_ROUTE_SEARCH_HPP_
