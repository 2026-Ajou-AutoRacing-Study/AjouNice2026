#pragma once

#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::behavior_velocity_planner
{
// Compare before interpolation. The existing BPP terminal stop must not be
// interpreted as a new traffic/obstacle stop imposed by velocity planning.
inline bool hasNewBehaviorStop(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & input,
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & output,
  const geometry_msgs::Point & ego)
{
  if (input.empty() || output.empty()) return true;
  for (const auto & candidate : output) {
    const auto & point = candidate.point;
    if (std::abs(point.longitudinal_velocity_mps) > 0.01 ||
        std::hypot(point.pose.position.x - ego.x, point.pose.position.y - ego.y) > 15.0) {
      continue;
    }
    const auto nearest = std::min_element(input.begin(), input.end(), [&](const auto & a, const auto & b) {
      return std::hypot(a.point.pose.position.x - point.pose.position.x,
                        a.point.pose.position.y - point.pose.position.y) <
             std::hypot(b.point.pose.position.x - point.pose.position.x,
                        b.point.pose.position.y - point.pose.position.y);
    });
    if (nearest->point.longitudinal_velocity_mps > 0.1) return true;
  }
  return false;
}
}  // namespace autoware::behavior_velocity_planner
