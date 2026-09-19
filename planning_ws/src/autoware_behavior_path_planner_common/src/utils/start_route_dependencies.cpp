// Full original route-extension implementations restored from source-closure-001.
// Copyright 2022 TIER IV, Inc. Apache License 2.0.
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <numeric>
namespace autoware::behavior_path_planner::utils
{
using RouteHandler = autoware::route_handler::RouteHandler;
using Pose = geometry_msgs::Pose;
lanelet::ConstLanelets extendNextLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route)
{
  if (lanes.empty()) return lanes;

  const auto next_lanes = route_handler->getNextLanelets(lanes.back());
  if (next_lanes.empty()) return lanes;

  // Add next lane
  auto extended_lanes = lanes;
  std::optional<lanelet::ConstLanelet> target_next_lane;
  for (const auto & next_lane : next_lanes) {
    // skip overlapping lanes
    if (next_lane.id() == lanes.front().id()) continue;

    // if route lane, set target and break
    if (route_handler->isRouteLanelet(next_lane)) {
      target_next_lane = next_lane;
      break;
    }
    if (!only_in_route && !target_next_lane) target_next_lane = next_lane;
  }
  if (target_next_lane) {
    extended_lanes.push_back(*target_next_lane);
  }
  return extended_lanes;
}

lanelet::ConstLanelets extendPrevLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route)
{
  if (lanes.empty()) return lanes;

  const auto prev_lanes = route_handler->getPreviousLanelets(lanes.front());
  if (prev_lanes.empty()) return lanes;

  // Add previous lane
  auto extended_lanes = lanes;
  std::optional<lanelet::ConstLanelet> target_prev_lane;
  for (const auto & prev_lane : prev_lanes) {
    // skip overlapping lanes
    if (prev_lane.id() == lanes.back().id()) continue;

    // if route lane, set target and break
    if (route_handler->isRouteLanelet(prev_lane)) {
      target_prev_lane = prev_lane;
      break;
    }
    if (!only_in_route && !target_prev_lane) target_prev_lane = prev_lane;
  }
  if (target_prev_lane) {
    extended_lanes.insert(extended_lanes.begin(), *target_prev_lane);
  }
  return extended_lanes;
}


lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length,
  const double forward_length, const bool forward_only_in_route)
{
  auto lanes = getCurrentLanes(planner_data);
  if (lanes.empty()) return lanes;
  const auto start_lane = lanes.front();

  double forward_length_sum = 0.0;
  double backward_length_sum = 0.0;

  while (backward_length_sum < backward_length) {
    auto extended_lanes = extendPrevLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for backward_length,
    // the extending process will not finish.
    if (extended_lanes.front().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      backward_length_sum += lanelet::geometry::length2d(extended_lanes.front());
    } else {
      break;  // no more previous lanes to add
    }
    lanes = extended_lanes;
  }

  while (forward_length_sum < forward_length) {
    auto extended_lanes = extendNextLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for forward_length,
    // the extending process will not finish.
    if (extended_lanes.back().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      forward_length_sum += lanelet::geometry::length2d(extended_lanes.back());
    } else {
      break;  // no more next lanes to add
    }

    // stop extending when the lane outside of the route is reached
    // if forward_length is a very large value, set it to true,
    // as it may continue to extend forever.
    if (forward_only_in_route) {
      if (!planner_data->route_handler->isRouteLanelet(extended_lanes.back())) {
        return lanes;
      }
    }

    lanes = extended_lanes;
  }

  return lanes;
}

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  return extendLanes(planner_data->route_handler, getCurrentLanes(planner_data));
}


lanelet::ConstLanelets getBackwardLanelets(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const double backward_length)
{
  const auto preceding_lanes =
    getPrecedingLanelets(route_handler, target_lanes, current_pose, backward_length);
  const auto calc_sum = [](size_t sum, const auto & lanes) { return sum + lanes.size(); };
  const auto num_of_lanes =
    std::accumulate(preceding_lanes.begin(), preceding_lanes.end(), 0u, calc_sum);

  lanelet::ConstLanelets backward_lanes{};
  backward_lanes.reserve(num_of_lanes);

  for (const auto & lanes : preceding_lanes) {
    backward_lanes.insert(backward_lanes.end(), lanes.begin(), lanes.end());
  }

  return backward_lanes;
}


lanelet::ConstLanelets combineLanelets(
  const lanelet::ConstLanelets & base_lanes, const lanelet::ConstLanelets & added_lanes)
{
  lanelet::ConstLanelets combined_lanes = base_lanes;
  for (const auto & added_lane : added_lanes) {
    const auto it = std::find_if(
      combined_lanes.begin(), combined_lanes.end(),
      [&added_lane](const lanelet::ConstLanelet & lane) { return lane.id() == added_lane.id(); });
    if (it == combined_lanes.end()) {
      combined_lanes.push_back(added_lane);
    }
  }

  return combined_lanes;
}

}
