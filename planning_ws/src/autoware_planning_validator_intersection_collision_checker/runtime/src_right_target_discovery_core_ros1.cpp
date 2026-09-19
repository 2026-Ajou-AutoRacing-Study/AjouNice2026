#include "autoware/planning_validator_intersection_collision_checker/right_target_discovery_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/right_target_filter_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/trajectory_overlap_core_ros1.hpp"

#include <algorithm>

namespace autoware::planning_validator::intersection_collision_checker
{
lanelet::ConstLanelets discover_right_target_lanelets(
  const lanelet::LaneletMap & map, const autoware_planning_msgs::Trajectory & trajectory,
  const LaneletSelection & ego_lanes, const bool check_turning_lanes,
  const bool check_crossing_lanes, const double crossing_angle_threshold,
  const std::size_t ego_front_index)
{
  if (ego_lanes.turn_lanelets.empty() || trajectory.points.size() < 2U) return {};
  const auto & first_turn_lane = ego_lanes.turn_lanelets.front();
  if (!first_turn_lane.hasAttribute("turn_direction") ||
      first_turn_lane.attribute("turn_direction").value() != "right") {
    return {};
  }
  if (first_turn_lane.centerline2d().size() < 2U) return {};

  const auto centerline = first_turn_lane.centerline2d();
  const auto [min_x, max_x] = std::minmax_element(
    centerline.begin(), centerline.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.x() < rhs.x(); });
  const auto [min_y, max_y] = std::minmax_element(
    centerline.begin(), centerline.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.y() < rhs.y(); });
  const lanelet::BoundingBox2d envelope(
    lanelet::BasicPoint2d(min_x->x(), min_y->y()),
    lanelet::BasicPoint2d(max_x->x(), max_y->y()));

  lanelet::ConstLanelets targets;
  for (const auto & candidate : map.laneletLayer.search(envelope)) {
    if (!eligible_right_target(
          candidate, ego_lanes, check_turning_lanes, check_crossing_lanes,
          crossing_angle_threshold)) {
      continue;
    }
    if (!trajectory_overlap_for_lanelet(candidate, trajectory, ego_front_index).has_value()) {
      continue;
    }
    targets.push_back(candidate);
  }
  return targets;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
