// Copyright 2020 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0

#ifndef AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_DEBUG_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_DEBUG_HPP_

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::debug
{
visualization_msgs::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::Polygon & polygon, const std::string & ns, int64_t module_id,
  const ros::Time & now, double x, double y, double z, double r, double g, double b);
visualization_msgs::MarkerArray createPathMarkerArray(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const std::string & ns,
  int64_t lane_id, const ros::Time & now, double x, double y, double z, double r, double g,
  double b);
visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::PredictedObjects & objects, const std::string & ns,
  int64_t module_id, const ros::Time & now, double r, double g, double b);
visualization_msgs::MarkerArray createPointsMarkerArray(
  const std::vector<geometry_msgs::Point> & points, const std::string & ns, int64_t module_id,
  const ros::Time & now, double x, double y, double z, double r, double g, double b);
}  // namespace autoware::behavior_velocity_planner::debug

#endif
