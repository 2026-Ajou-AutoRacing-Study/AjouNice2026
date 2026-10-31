// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#ifndef OBSTACLE_CRUISE_TRAJECTORY_INDEX_MAPPING_HPP_
#define OBSTACLE_CRUISE_TRAJECTORY_INDEX_MAPPING_HPP_

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_planning_msgs/TrajectoryPoint.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner
{
// Cropping changes the origin and resampling changes the spacing (and may insert
// a stop point). Match the collision sample's pose back to the original array;
// neither adding an offset nor multiplying an index preserves that correspondence.
// Heading disambiguates intersecting/retraced segments with different directions.
inline std::optional<std::size_t> map_collision_index_to_original(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & original,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & collision_trajectory,
  const std::size_t collision_index)
{
  if (original.empty() || collision_index >= collision_trajectory.size()) {
    return std::nullopt;
  }
  const auto & pose = collision_trajectory.at(collision_index).pose;
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) {
    return std::nullopt;
  }
  return autoware::motion_utils::findNearestIndex(
    original, pose, std::numeric_limits<double>::max(), M_PI_2);
}
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_CRUISE_TRAJECTORY_INDEX_MAPPING_HPP_
