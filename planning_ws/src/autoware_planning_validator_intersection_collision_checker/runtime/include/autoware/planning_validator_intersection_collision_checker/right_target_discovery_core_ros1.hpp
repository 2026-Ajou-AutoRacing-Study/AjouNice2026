#pragma once

#include "autoware/planning_validator_intersection_collision_checker/lanelet_selection_core_ros1.hpp"

#include <autoware_planning_msgs/Trajectory.h>
#include <lanelet2_core/LaneletMap.h>

#include <cstddef>

namespace autoware::planning_validator::intersection_collision_checker
{
// ROS1 rendering of the candidate-map/filter/trajectory-overlap part of
// utils.cpp:set_right_turn_target_lanelets.  Time-window, signal-priority and
// lane-extension ownership stay with the integration layer.
lanelet::ConstLanelets discover_right_target_lanelets(
  const lanelet::LaneletMap & map, const autoware_planning_msgs::Trajectory & trajectory,
  const LaneletSelection & ego_lanes, bool check_turning_lanes, bool check_crossing_lanes,
  double crossing_angle_threshold, std::size_t ego_front_index = 0U);
}  // namespace autoware::planning_validator::intersection_collision_checker
