#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAFFIC_LIGHT_UTILS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAFFIC_LIGHT_UTILS_HPP_

#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <lanelet2_core/Forward.h>

#include <limits>
#include <memory>
#include <optional>

namespace autoware::behavior_path_planner::utils::traffic_light
{

double getDistanceToNextTrafficLight(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);

std::optional<double> calcDistanceToRedTrafficLight(
  const lanelet::ConstLanelets & lanelets,
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data);

bool isStoppedAtRedTrafficLightWithinDistance(
  const lanelet::ConstLanelets & lanelets,
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  double distance_threshold = std::numeric_limits<double>::infinity());

// The traffic policy decision remains owned by the staged provider. This BPP
// surface associates a regulatory-element id with fresh PlannerData state.
bool isTrafficSignalStop(
  const lanelet::ConstLanelets & lanelets,
  const std::shared_ptr<const PlannerData> & planner_data);

}  // namespace autoware::behavior_path_planner::utils::traffic_light

#endif
