#include <autoware/behavior_path_goal_planner_module/goal_candidate_update_core.hpp>

#include <autoware/behavior_path_goal_planner_module/goal_candidate_ranker.hpp>

namespace autoware::behavior_path_planner
{
void updateGoalCandidates(
  GoalCandidates * candidates, const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & current_lanes,
  const autoware_internal_planning_msgs::PathWithLaneId & current_center_line_path,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateUpdateParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
{
  if (!candidates) return;
  if (parameters.prioritize_goals_before_objects) {
    countGoalCandidateObjectsToAvoid(
      candidates, objects, current_lanes, current_center_line_path, local_vehicle_footprint,
      parameters.object_collision_margin);
  }
  rankGoalCandidates(
    candidates, parameters.goal_priority,
    parameters.minimum_weighted_distance_lateral_weight,
    parameters.prioritize_goals_before_objects);
  updateGoalCandidateSafety(
    candidates, local_vehicle_footprint, objects, parameters.safety, occupancy_grid_map);
}
}  // namespace autoware::behavior_path_planner
