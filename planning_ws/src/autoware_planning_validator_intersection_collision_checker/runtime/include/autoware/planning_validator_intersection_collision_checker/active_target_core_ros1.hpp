#pragma once

#include "autoware/planning_validator_intersection_collision_checker/trajectory_overlap_core_ros1.hpp"

#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator::intersection_collision_checker
{
struct ActiveTarget
{
  lanelet::Id id{};
  lanelet::ConstLanelets lanelets;
  geometry_msgs::Pose overlap_pose;
  TrajectoryOverlap overlap;
};

// Shared source record assembly after a right/left candidate is found. A
// fully-prioritized ego signal removes an existing candidate in the source;
// represented here as no active target rather than a fabricated empty record.
std::optional<ActiveTarget> make_active_target(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelet & candidate, const autoware_planning_msgs::Trajectory & trajectory,
  std::size_t ego_front_index, double detection_range, bool ego_fully_prioritized);
}  // namespace autoware::planning_validator::intersection_collision_checker
