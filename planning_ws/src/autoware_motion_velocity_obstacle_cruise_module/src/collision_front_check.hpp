#ifndef AUTOWARE_CRUISE_COLLISION_FRONT_CHECK_HPP_
#define AUTOWARE_CRUISE_COLLISION_FRONT_CHECK_HPP_

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <cmath>

namespace autoware::motion_velocity_planner
{
// collision_index belongs to collision_trajectory (the decimated polygon source),
// never to the original full-resolution trajectory. Keep the native signed-arc rule.
template<class Points, class Position>
bool collision_is_ahead_of_object(
  const Points & collision_trajectory, const Position & object_position,
  const size_t collision_index, const double object_radius)
{
  if (collision_trajectory.empty() || collision_index >= collision_trajectory.size() ||
      !std::isfinite(object_radius) || object_radius < 0.0) return false;
  const auto object_index = autoware::motion_utils::findNearestIndex(
    collision_trajectory, object_position);
  const double distance = autoware::motion_utils::calcSignedArcLength(
    collision_trajectory, object_index, collision_index);
  return std::isfinite(distance) && distance > -object_radius;
}
}  // namespace autoware::motion_velocity_planner
#endif
