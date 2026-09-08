#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_UPDATE_CORE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_UPDATE_CORE_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate_object_counter.hpp>
#include <autoware/behavior_path_goal_planner_module/goal_candidate_safety_core.hpp>

#include <string>

namespace autoware::behavior_path_planner
{
struct GoalCandidateUpdateParameters
{
  bool prioritize_goals_before_objects{false};
  std::string goal_priority{};
  double minimum_weighted_distance_lateral_weight{0.0};
  double object_collision_margin{0.0};
  GoalCandidateSafetyParameters safety{};
};

// Source GoalSearcher update order after external dynamic-object extraction:
// optional object count, candidate ranking, then candidate safety evaluation.
void updateGoalCandidates(
  GoalCandidates * candidates, const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & current_lanes,
  const autoware_internal_planning_msgs::PathWithLaneId & current_center_line_path,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateUpdateParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_UPDATE_CORE_HPP_
