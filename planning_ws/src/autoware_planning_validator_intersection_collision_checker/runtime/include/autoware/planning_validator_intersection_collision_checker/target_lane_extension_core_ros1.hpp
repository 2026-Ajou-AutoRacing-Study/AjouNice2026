#pragma once

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator::intersection_collision_checker
{
// Exact source extension direction: target lane followed by predecessor
// lanes, returned in driving order from the oldest predecessor to target.
lanelet::ConstLanelets extend_target_lanelet(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelet & target_lanelet, const geometry_msgs::Pose & overlap_pose,
  double detection_range);
}  // namespace autoware::planning_validator::intersection_collision_checker
