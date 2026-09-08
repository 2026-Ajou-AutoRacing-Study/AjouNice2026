#pragma once

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>

#include <memory>

namespace autoware::behavior_velocity_planner
{
bool smoothPath(
  const autoware_internal_planning_msgs::PathWithLaneId & in_path,
  autoware_internal_planning_msgs::PathWithLaneId & out_path,
  const std::shared_ptr<const PlannerData> & planner_data);
bool smoothPath(
  const autoware_internal_planning_msgs::PathWithLaneId & in_path,
  autoware_internal_planning_msgs::PathWithLaneId & out_path, const PlannerData & planner_data);
}  // namespace autoware::behavior_velocity_planner
