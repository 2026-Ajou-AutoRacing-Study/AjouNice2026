#ifndef AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_PATH_UTILIZATION_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_PATH_UTILIZATION_HPP_

#include <autoware_planning_msgs/Path.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <string>

namespace autoware::behavior_velocity_planner
{
bool splineInterpolate(
  const autoware_internal_planning_msgs::PathWithLaneId & input, double interval,
  autoware_internal_planning_msgs::PathWithLaneId & output, const std::string & logger);
autoware_planning_msgs::Path interpolatePath(
  const autoware_planning_msgs::Path & path, double length, double interval);
autoware_planning_msgs::Path filterLitterPathPoint(const autoware_planning_msgs::Path & path);
autoware_planning_msgs::Path filterStopPathPoint(const autoware_planning_msgs::Path & path);
}  // namespace autoware::behavior_velocity_planner

#endif
