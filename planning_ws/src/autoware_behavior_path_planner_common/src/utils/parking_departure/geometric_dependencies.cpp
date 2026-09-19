// Restored from the frozen full BPP-common utils.cpp and parking_departure/utils.cpp.
// Copyright 2022 TIER IV, Inc. Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <ros/console.h>

namespace autoware::behavior_path_planner::utils
{
double getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::Pose & pose, bool left_side)
{
  const auto closest_lanelet_opt = experimental::lanelet2_utils::get_closest_lanelet(lanelets, pose);
  if (closest_lanelet_opt) {
    return getSignedDistanceFromLaneBoundary(closest_lanelet_opt.value(), pose.position, left_side);
  }
  ROS_ERROR_NAMED("behavior_path_planner.utils", "closest lanelet not found.");
  return 0.0;
}
namespace parking_departure
{
std::pair<double, bool> calcEndArcLength(
  const double s_start, const double forward_path_length, const lanelet::ConstLanelets & road_lanes,
  const geometry_msgs::Pose & goal_pose)
{
  const double s_forward_length = s_start + forward_path_length;
  if (!utils::isInLanelets(goal_pose, road_lanes)) return {s_forward_length, false};
  const double s_goal = experimental::lanelet2_utils::get_arc_coordinates(road_lanes, goal_pose).length;
  if (s_goal < s_start || s_goal >= s_forward_length) return {s_forward_length, false};
  return {s_goal, true};
}
}
}
