#pragma once
#include "autoware/planning_validator_intersection_collision_checker/lanelet_selection_core_ros1.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
namespace autoware::planning_validator::intersection_collision_checker {
bool eligible_right_target(
  const lanelet::ConstLanelet & candidate, const LaneletSelection & ego_lanes,
  bool check_turning_lanes, bool check_crossing_lanes, double crossing_angle_threshold);
}
