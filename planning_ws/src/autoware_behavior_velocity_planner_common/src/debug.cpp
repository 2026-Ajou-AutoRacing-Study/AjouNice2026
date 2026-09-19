// Copyright 2020 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <algorithm>

namespace autoware::behavior_velocity_planner::debug
{
using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_color;
using autoware_utils_visualization::create_marker_scale;

visualization_msgs::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::Polygon & polygon, const std::string & ns, const int64_t module_id,
  const ros::Time & now, const double x, const double y, const double z, const double r,
  const double g, const double b)
{
  visualization_msgs::MarkerArray output;
  auto marker = create_default_marker(
    "map", now, ns, static_cast<int32_t>(module_id), visualization_msgs::Marker::LINE_STRIP,
    create_marker_scale(x, y, z), create_marker_color(r, g, b, 0.8F));
  marker.lifetime = ros::Duration(0.3);
  for (const auto & point : polygon.points) {
    geometry_msgs::Point output_point;
    output_point.x = point.x;
    output_point.y = point.y;
    output_point.z = point.z;
    marker.points.push_back(output_point);
  }
  if (!marker.points.empty()) marker.points.push_back(marker.points.front());
  output.markers.push_back(marker);
  return output;
}

visualization_msgs::MarkerArray createPathMarkerArray(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const ros::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b)
{
  visualization_msgs::MarkerArray output;
  for (size_t index = 0; index < path.points.size(); ++index) {
    const auto & point = path.points.at(index);
    auto marker = create_default_marker(
      "map", now, ns, static_cast<int32_t>(planning_utils::bitShift(lane_id) + index),
      visualization_msgs::Marker::ARROW, create_marker_scale(x, y, z),
      create_marker_color(r, g, b, 0.999F));
    marker.lifetime = ros::Duration(0.3);
    marker.pose = point.point.pose;
    if (std::find(point.lane_ids.begin(), point.lane_ids.end(), lane_id) == point.lane_ids.end()) {
      marker.color = create_marker_color(0.5F, 0.5F, 0.5F, 0.999F);
    }
    output.markers.push_back(marker);
  }
  return output;
}

visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::PredictedObjects & objects, const std::string & ns,
  const int64_t module_id, const ros::Time & now, const double r, const double g, const double b)
{
  visualization_msgs::MarkerArray output;
  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0),
    create_marker_color(r, g, b, 0.8F));
  marker.lifetime = ros::Duration(1.0);
  for (size_t index = 0; index < objects.objects.size(); ++index) {
    marker.id = static_cast<int>(planning_utils::bitShift(module_id) + index);
    marker.pose = objects.objects.at(index).kinematics.initial_pose_with_covariance.pose;
    output.markers.push_back(marker);
  }
  return output;
}

visualization_msgs::MarkerArray createPointsMarkerArray(
  const std::vector<geometry_msgs::Point> & points, const std::string & ns, const int64_t module_id,
  const ros::Time & now, const double x, const double y, const double z, const double r,
  const double g, const double b)
{
  visualization_msgs::MarkerArray output;
  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::Marker::SPHERE, create_marker_scale(x, y, z),
    create_marker_color(r, g, b, 0.999F));
  marker.lifetime = ros::Duration(0.3);
  for (size_t index = 0; index < points.size(); ++index) {
    marker.id = static_cast<int32_t>(index + planning_utils::bitShift(module_id));
    marker.pose.position = points.at(index);
    output.markers.push_back(marker);
  }
  return output;
}
}  // namespace autoware::behavior_velocity_planner::debug
