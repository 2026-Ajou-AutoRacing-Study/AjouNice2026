#pragma once

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
struct LaneletSelection
{
  lanelet::ConstLanelets turn_lanelets;
  lanelet::ConstLanelets trajectory_lanelets;
  lanelet::ConstLanelets connected_lanelets;
};

// ROS1 rendering of utils.cpp:set_trajectory_lanelets.  Unlike a nearest-map
// query, this deliberately accepts only a RouteHandler-ready route lanelet.
std::optional<LaneletSelection> select_trajectory_lanelets(
  const autoware_planning_msgs::Trajectory & trajectory,
  const autoware::route_handler::RouteHandler & route_handler,
  const geometry_msgs::Pose & ego_pose);
}  // namespace autoware::planning_validator::intersection_collision_checker
