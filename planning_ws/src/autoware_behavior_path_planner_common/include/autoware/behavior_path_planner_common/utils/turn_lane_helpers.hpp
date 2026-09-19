// Copyright 2025 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <autoware/behavior_path_planner_common/utils/root_reference_utils.hpp>
namespace autoware::behavior_path_planner::utils {
bool is_turn_direction_lane(const std::string & attribute);
bool is_turn_direction_lane(const lanelet::ConstLanelet & lane);
lanelet::ConstLanelets nearest_turn_direction_lane_within_route(
  const Pose &,const RouteHandler &,const lanelet::ConstLanelets &);
std::optional<double> calc_distance_to_next_turn_direction_lane(
  const Pose &,const RouteHandler &,const lanelet::ConstLanelets &,const std::string &);
}
