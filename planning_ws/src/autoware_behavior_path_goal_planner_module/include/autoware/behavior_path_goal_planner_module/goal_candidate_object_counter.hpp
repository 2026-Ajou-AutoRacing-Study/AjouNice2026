#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_OBJECT_COUNTER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_OBJECT_COUNTER_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <lanelet2_core/primitives/Lanelet.h>

namespace autoware::behavior_path_planner
{

// Source GoalSearcher countObjectsToAvoid loop after it has obtained the
// current centerline path and current lane sequence from PlannerData.
void countGoalCandidateObjectsToAvoid(
  GoalCandidates * candidates, const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & current_lanes,
  const autoware_internal_planning_msgs::PathWithLaneId & current_center_line_path,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  double object_collision_margin);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_OBJECT_COUNTER_HPP_
