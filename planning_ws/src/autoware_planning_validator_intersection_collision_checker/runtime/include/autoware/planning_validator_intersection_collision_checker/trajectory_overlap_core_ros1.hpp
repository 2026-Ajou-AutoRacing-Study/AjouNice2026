#pragma once

#include <autoware_planning_msgs/Trajectory.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <cstddef>
#include <optional>

namespace autoware::planning_validator::intersection_collision_checker
{
struct TrajectoryOverlap
{
  std::size_t first_index{};
  std::size_t last_index{};
  double first_time_seconds{};
  double last_time_seconds{};
};

// Source-shaped rendering of utils.cpp:get_overlap_index plus the target-lane
// time extraction. No result means no polygon overlap, or an overlap fully
// behind the ego-front trajectory index.
std::optional<TrajectoryOverlap> trajectory_overlap_for_lanelet(
  const lanelet::ConstLanelet & lanelet, const autoware_planning_msgs::Trajectory & trajectory,
  std::size_t ego_front_index);
}  // namespace autoware::planning_validator::intersection_collision_checker
