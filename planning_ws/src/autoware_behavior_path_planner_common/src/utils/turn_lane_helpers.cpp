// Copyright 2025 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_planner_common/utils/turn_lane_helpers.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <algorithm>
#include <numeric>
namespace autoware::behavior_path_planner::utils {
bool is_turn_direction_lane(const std::string & lanelet_attribute_string)
{
  return lanelet_attribute_string == "left" || lanelet_attribute_string == "right";
}

bool is_turn_direction_lane(const lanelet::ConstLanelet & lanelet)
{
  const std::string turn_direction = lanelet.attributeOr("turn_direction", "else");
  return is_turn_direction_lane(turn_direction);
}

lanelet::ConstLanelets nearest_turn_direction_lane_within_route(
  const Pose & current_pose, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return {};
  }

  const auto current_lanelet_opt =
    experimental::lanelet2_utils::get_closest_lanelet(lanelets, current_pose);
  if (!current_lanelet_opt) {
    return {};
  }
  const auto & current_lanelet = current_lanelet_opt.value();

  const auto current_llt_itr = std::find_if(
    lanelets.begin(), lanelets.end(),
    [&current_lanelet](const auto & llt) { return llt.id() == current_lanelet.id(); });

  if (current_llt_itr == lanelets.end()) {
    return {};
  }

  const auto is_current_lane = [&current_llt_itr](const auto & llt) {
    return llt.id() == current_llt_itr->id();
  };
  const auto is_in_turn_direction_lane = [&current_pose](const auto & llt) {
    const auto & position = current_pose.position;
    const lanelet::BasicPoint2d point(position.x, position.y);
    return lanelet::geometry::inside(llt, point);
  };

  lanelet::ConstLanelets turn_direction_lanes;
  const auto prev_lanes = route_handler.getPreviousLanelets(*current_llt_itr);

  // do a reverse search
  for (const auto & lane : prev_lanes) {
    const auto next_lanes = route_handler.getNextLanelets(lane);

    const auto is_connected_to_current_lane =
      std::any_of(next_lanes.begin(), next_lanes.end(), is_current_lane);

    if (!is_connected_to_current_lane) {
      continue;
    }

    if (is_turn_direction_lane(lane) && is_in_turn_direction_lane(lane)) {
      turn_direction_lanes.push_back(lane);
    }
  }

  return turn_direction_lanes;
}

std::optional<double> calc_distance_to_next_turn_direction_lane(
  const Pose & current_pose, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelets, const std::string & shift_direction_str)
{
  if (shift_direction_str != "left" && shift_direction_str != "right") {
    return std::nullopt;
  }

  const auto current_lanelet_opt =
    experimental::lanelet2_utils::get_closest_lanelet(lanelets, current_pose);
  if (!current_lanelet_opt) {
    return std::nullopt;
  }
  const auto & current_lanelet = current_lanelet_opt.value();

  const auto current_llt_itr = std::find_if(
    lanelets.begin(), lanelets.end(),
    [&current_lanelet](const auto & llt) { return llt.id() == current_lanelet.id(); });

  // case 1: current lanelet is not in lanelets
  if (current_llt_itr == lanelets.end()) {
    return std::nullopt;
  }

  const auto is_same_direction_shift_and_turn_lane = [&shift_direction_str](const auto & llt) {
    const std::string turn_direction = llt.attributeOr("turn_direction", "else");
    return turn_direction == shift_direction_str;
  };

  // case 2: current lanelet is in lanelets. We also search the succeeding lane after final lane.
  const auto nearest_turn_llt_itr =
    std::find_if(current_llt_itr, lanelets.end(), [&](const auto & llt) {
      const auto next_lanelets = route_handler.getNextLanelets(llt);
      return std::any_of(
        next_lanelets.begin(), next_lanelets.end(), is_same_direction_shift_and_turn_lane);
    });

  if (nearest_turn_llt_itr == lanelets.end()) {
    return std::nullopt;
  }

  const auto distance_covered =
    autoware::experimental::lanelet2_utils::get_arc_coordinates({*current_llt_itr}, current_pose)
      .length;
  const auto remaining_dist_on_current_lane =
    lanelet::geometry::length3d(*current_llt_itr) - distance_covered;
  const auto dist_to_next_turn_direction_lane = std::accumulate(
    std::next(current_llt_itr), std::next(nearest_turn_llt_itr), 0.0,
    [](const auto & sum, const auto & llt) { return sum + lanelet::geometry::length3d(llt); });

  return remaining_dist_on_current_lane + dist_to_next_turn_direction_lane;
}


}
