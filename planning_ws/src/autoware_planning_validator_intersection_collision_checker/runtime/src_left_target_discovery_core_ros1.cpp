#include "autoware/planning_validator_intersection_collision_checker/left_target_discovery_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/trajectory_overlap_core_ros1.hpp"

namespace autoware::planning_validator::intersection_collision_checker
{
namespace
{
bool ignored_turn_candidate(const lanelet::ConstLanelet & lane, const bool check_turning_lanes)
{
  return lane.hasAttribute("turn_direction") &&
         lane.attribute("turn_direction").value() != "straight" && !check_turning_lanes;
}
}  // namespace

lanelet::ConstLanelets discover_left_target_lanelets(
  const lanelet::ConstLanelets & predecessor_candidates, const lanelet::Id last_turn_lane_id,
  const autoware_planning_msgs::Trajectory & trajectory, const std::size_t ego_front_index,
  const bool check_turning_lanes)
{
  lanelet::ConstLanelets targets;
  for (const auto & candidate : predecessor_candidates) {
    if (candidate.id() == last_turn_lane_id ||
        ignored_turn_candidate(candidate, check_turning_lanes)) {
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
