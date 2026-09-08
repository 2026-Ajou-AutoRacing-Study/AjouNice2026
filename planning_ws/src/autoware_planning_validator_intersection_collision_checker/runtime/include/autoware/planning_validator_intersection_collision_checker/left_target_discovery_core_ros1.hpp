#pragma once

#include <autoware_planning_msgs/Trajectory.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <cstddef>

namespace autoware::planning_validator::intersection_collision_checker
{
// Render the source loop after RouteHandler::getPreviousLanelets(next_lanelet).
// Route topology resolution remains at the RouteHandler integration boundary;
// this core preserves the candidate exclusion and future-overlap rules.
lanelet::ConstLanelets discover_left_target_lanelets(
  const lanelet::ConstLanelets & predecessor_candidates, lanelet::Id last_turn_lane_id,
  const autoware_planning_msgs::Trajectory & trajectory, std::size_t ego_front_index,
  bool check_turning_lanes);
}  // namespace autoware::planning_validator::intersection_collision_checker
