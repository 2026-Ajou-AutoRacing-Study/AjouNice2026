#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__UTILS_HPP_
#include <lanelet2_core/primitives/Lanelet.h>
#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <utility>

#include <autoware/behavior_path_planner_common/utils/parking_departure/common_module_data.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/velocity_utils.hpp>

#include <memory>
#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::parking_departure
{
std::optional<double> calcFeasibleDecelDistance(
  std::shared_ptr<const PlannerData> planner_data, double acc_lim, double jerk_lim,
  double target_velocity);
inline void updatePathProperty(
  const std::shared_ptr<path_safety_checker::EgoPredictedPathParams> & params,
  const std::pair<double, double> & terminal_velocity_and_acceleration)
{
  if (!params) return;
  params->max_velocity = terminal_velocity_and_acceleration.first;
  params->acceleration = std::max(
    terminal_velocity_and_acceleration.second, params->min_acceleration);
}

inline void initializeCollisionCheckDebugMap(
  path_safety_checker::CollisionCheckDebugMap & collision_check_debug_map)
{
  collision_check_debug_map.clear();
}

std::pair<double, bool> calcEndArcLength(
  double s_start, double forward_path_length, const lanelet::ConstLanelets & road_lanes,
  const geometry_msgs::Pose & goal_pose);

inline void updateSafetyCheckTargetObjectsData(
  StartGoalPlannerData & data, const autoware_perception_msgs::PredictedObjects & filtered_objects,
  const path_safety_checker::TargetObjectsOnLane & target_objects_on_lane,
  const std::vector<path_safety_checker::PoseWithVelocityStamped> & ego_predicted_path)
{
  data.filtered_objects = filtered_objects;
  data.target_objects_on_lane = target_objects_on_lane;
  data.ego_predicted_path = ego_predicted_path;
}
}
#endif
