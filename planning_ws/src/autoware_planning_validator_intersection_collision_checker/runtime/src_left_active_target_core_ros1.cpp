#include "autoware/planning_validator_intersection_collision_checker/left_active_target_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/left_target_discovery_core_ros1.hpp"

#include <autoware/route_handler/route_handler.hpp>

namespace autoware::planning_validator::intersection_collision_checker
{
std::vector<ActiveTarget> make_left_active_targets(
  const autoware::route_handler::RouteHandler & route_handler,
  const autoware_planning_msgs::Trajectory & front_trajectory,
  const LaneletSelection & ego_lanes, const std::size_t ego_front_index,
  const double time_horizon, const intersection_collision_checker_ros1::Params & params,
  const bool ego_fully_prioritized)
{
  std::vector<ActiveTarget> targets;
  if (ego_lanes.turn_lanelets.empty()) return targets;
  const auto & first_turn = ego_lanes.turn_lanelets.front();
  if (!first_turn.hasAttribute("turn_direction") ||
      first_turn.attribute("turn_direction").value() != "left") {
    return targets;
  }
  if (params.left_turn.check_traffic_signal && ego_fully_prioritized) return targets;

  const auto & last_turn = ego_lanes.turn_lanelets.back();
  lanelet::ConstLanelet next_lane;
  if (!route_handler.getNextLaneletWithinRoute(last_turn, &next_lane)) return targets;
  const auto candidates = discover_left_target_lanelets(
    route_handler.getPreviousLanelets(next_lane), last_turn.id(), front_trajectory,
    ego_front_index, params.left_turn.check_turning_lanes);
  for (const auto & candidate : candidates) {
    const auto target = make_active_target(
      route_handler, candidate, front_trajectory, ego_front_index, params.detection_range,
      false);
    if (!target.has_value() || target->overlap.first_time_seconds > time_horizon) continue;
    targets.push_back(target.value());
  }
  return targets;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
