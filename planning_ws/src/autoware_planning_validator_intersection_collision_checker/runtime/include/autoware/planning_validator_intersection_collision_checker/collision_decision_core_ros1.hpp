#pragma once

#include "autoware/planning_validator_intersection_collision_checker/parameters_ros1.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
struct CollisionObjectState
{
  bool is_reliable{false};
  bool is_moving{false};
  double ttc{std::numeric_limits<double>::infinity()};
};

// Exact decision branch inside IntersectionCollisionChecker::check_collision.
// Tracking and lanelet association remain separate cores; an object that has
// not met the source reliability requirement cannot invalidate trajectory.
inline bool is_colliding(
  const CollisionObjectState & object, const std::pair<double, double> & ego_overlap_time,
  const double ego_velocity, const Params & params)
{
  if (!object.is_reliable) return false;
  const double close_time_threshold =
    ego_velocity < params.filter.min_velocity ? params.close_time_th : std::abs(ego_velocity / params.ego_deceleration);
  if (object.ttc < std::numeric_limits<double>::epsilon() && ego_overlap_time.first < close_time_threshold) return true;
  if (object.is_moving) {
    const double time_gap = object.ttc > ego_overlap_time.second
      ? object.ttc - ego_overlap_time.second
      : (object.ttc < ego_overlap_time.first ? ego_overlap_time.first - object.ttc : 0.0);
    return time_gap < params.ttc_threshold;
  }
  return false;
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
