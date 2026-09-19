#pragma once

#include "autoware/planning_validator_intersection_collision_checker/active_target_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/lanelet_selection_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/parameters_ros1.hpp"

#include <autoware_planning_msgs/Trajectory.h>

#include <vector>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator::intersection_collision_checker
{
// Source-shaped right-turn target record stage after target discovery. Signal
// priority is passed explicitly from its fresh-regulatory-element boundary.
std::vector<ActiveTarget> make_right_active_targets(
  const autoware::route_handler::RouteHandler & route_handler,
  const autoware_planning_msgs::Trajectory & front_trajectory,
  const LaneletSelection & ego_lanes, std::size_t ego_front_index,
  double time_horizon, const intersection_collision_checker_ros1::Params & params,
  bool ego_fully_prioritized);
}  // namespace autoware::planning_validator::intersection_collision_checker
