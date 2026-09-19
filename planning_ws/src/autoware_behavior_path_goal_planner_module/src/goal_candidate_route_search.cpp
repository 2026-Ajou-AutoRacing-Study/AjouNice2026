#include <autoware/behavior_path_goal_planner_module/goal_candidate_route_search.hpp>

#include <autoware/behavior_path_goal_planner_module/pull_over_lane_adapter.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

namespace autoware::behavior_path_planner
{
GoalCandidates searchGoalCandidatesFromRoute(
  const std::shared_ptr<const autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & reference_goal_pose,
  const GoalCandidateSearchParameters & parameters,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateAreaConstraints & area_constraints)
{
  const auto pull_over_lanes = selectPullOverLanes(
    route_handler, reference_goal_pose, parameters.left_side_parking,
    parameters.backward_goal_search_length, parameters.forward_goal_search_length);
  if (!pull_over_lanes || pull_over_lanes->empty()) return {};
  const auto combined_lane =
    autoware::experimental::lanelet2_utils::combine_lanelets_shape(*pull_over_lanes);
  if (!combined_lane) return {};
  const auto boundary = parameters.left_side_parking
                          ? combined_lane->leftBound()
                          : combined_lane->rightBound();
  return searchGoalCandidatesAlongBoundary(
    boundary, reference_goal_pose, parameters, local_vehicle_footprint,
    appendGoalCandidateRegulatoryAreas(area_constraints, *pull_over_lanes));
}
}  // namespace autoware::behavior_path_planner
