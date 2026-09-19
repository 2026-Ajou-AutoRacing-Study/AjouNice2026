#include "autoware/planning_validator_intersection_collision_checker/right_active_target_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/right_target_discovery_core_ros1.hpp"

#include <autoware/route_handler/route_handler.hpp>

namespace autoware::planning_validator::intersection_collision_checker
{
std::vector<ActiveTarget> make_right_active_targets(
  const autoware::route_handler::RouteHandler & route_handler,
  const autoware_planning_msgs::Trajectory & front_trajectory,
  const LaneletSelection & ego_lanes, const std::size_t ego_front_index,
  const double time_horizon, const intersection_collision_checker_ros1::Params & params,
  const bool ego_fully_prioritized)
{
  std::vector<ActiveTarget> targets;
  if (params.right_turn.check_traffic_signal && ego_fully_prioritized) return targets;
  const auto map = route_handler.getLaneletMapPtr();
  if (!map) return targets;
  const auto candidates = discover_right_target_lanelets(
    *map, front_trajectory, ego_lanes, params.right_turn.check_turning_lanes,
    params.right_turn.check_crossing_lanes, params.right_turn.crossing_lane_angle_th,
    ego_front_index);
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
