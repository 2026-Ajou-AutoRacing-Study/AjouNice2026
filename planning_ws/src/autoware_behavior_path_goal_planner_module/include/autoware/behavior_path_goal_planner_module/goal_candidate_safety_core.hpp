#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_SAFETY_CORE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_SAFETY_CORE_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate.hpp>
#include <autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <geometry_msgs/Pose.h>

#include <memory>

namespace autoware::behavior_path_planner
{

// Explicit inputs for the source GoalSearcher collision checks.  A non-null
// detector is required whenever either occupancy-grid option is enabled, as in
// the source module which dereferences its supplied detector in that branch.
struct GoalCandidateSafetyParameters
{
  bool use_occupancy_grid_for_goal_search{false};
  bool use_occupancy_grid_for_goal_longitudinal_margin{false};
  double occupancy_grid_collision_check_margin{0.0};
  double longitudinal_margin{0.0};
  double object_recognition_collision_check_hard_margin{0.0};
  double vehicle_width{0.0};
  double base_link2front{0.0};
  double base_link2rear{0.0};
};

bool checkGoalCandidateCollision(
  const autoware_utils_geometry::LinearRing2d & vehicle_footprint,
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::PredictedObjects & objects,
  const GoalCandidateSafetyParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map);

bool checkGoalCandidateLongitudinalMargin(
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::PredictedObjects & objects,
  const GoalCandidateSafetyParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map);

void updateGoalCandidateSafety(
  GoalCandidates * candidates, const autoware_utils_geometry::LinearRing2d & vehicle_footprint,
  const autoware_perception_msgs::PredictedObjects & objects,
  const GoalCandidateSafetyParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_SAFETY_CORE_HPP_
