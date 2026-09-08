#include "autoware/planning_validator_intersection_collision_checker/right_target_filter_core_ros1.hpp"
#include <algorithm>
#include <cmath>
namespace autoware::planning_validator::intersection_collision_checker {
namespace {
bool contains(const lanelet::ConstLanelets & lanes, lanelet::Id id) {
  return std::any_of(lanes.begin(), lanes.end(), [id](const auto & l) { return l.id() == id; });
}
bool is_turn(const lanelet::ConstLanelet & lane) {
  return lane.hasAttribute("turn_direction") && lane.attribute("turn_direction").value() != "straight";
}
bool crossing(const lanelet::ConstLanelet & lane, const lanelet::ConstLanelet & ref, double threshold) {
  const auto a = lane.centerline2d(); const auto b = ref.centerline2d();
  if (a.size() < 2U || b.size() < 2U) return false;
  const Eigen::Vector2d av = (a.front().basicPoint2d() - a.back().basicPoint2d()).normalized();
  // Keep the ROS2 reference direction exactly: it deliberately uses the
  // first centerline segment of the ego turn lane, not its overall chord.
  const Eigen::Vector2d bv =
    (b.front().basicPoint2d() - std::next(b.begin())->basicPoint2d()).normalized();
  return std::abs(av.dot(bv)) < std::abs(std::cos(threshold));
}
}
bool eligible_right_target(
  const lanelet::ConstLanelet & candidate, const LaneletSelection & ego_lanes,
  bool check_turning_lanes, bool check_crossing_lanes, double crossing_angle_threshold) {
  if (!candidate.hasAttribute(lanelet::AttributeName::Subtype) ||
      candidate.attribute(lanelet::AttributeName::Subtype).value() != lanelet::AttributeValueString::Road) return false;
  if (contains(ego_lanes.trajectory_lanelets, candidate.id()) || contains(ego_lanes.connected_lanelets, candidate.id())) return false;
  if (is_turn(candidate) && !check_turning_lanes) return false;
  return ego_lanes.turn_lanelets.empty() || check_crossing_lanes ||
    !crossing(candidate, ego_lanes.turn_lanelets.front(), crossing_angle_threshold);
}
}
