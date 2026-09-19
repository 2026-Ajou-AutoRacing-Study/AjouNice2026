// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware_utils_visualization/marker_helper.hpp"

namespace autoware_utils_visualization
{
visualization_msgs::Marker create_default_marker(
  const std::string & frame_id, const ros::Time & now, const std::string & ns, int32_t id,
  int32_t type, const geometry_msgs::Vector3 & scale, const std_msgs::ColorRGBA & color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = now;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(MARKER_LIFETIME);
  marker.pose.position = create_marker_position(0.0, 0.0, 0.0);
  marker.pose.orientation = create_marker_orientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = scale;
  marker.color = color;
  marker.frame_locked = true;
  return marker;
}

visualization_msgs::Marker create_deleted_default_marker(
  const ros::Time & now, const std::string & ns, int32_t id)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = now;
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::Marker::DELETE;
  return marker;
}

void append_marker_array(
  const visualization_msgs::MarkerArray & additional_marker_array,
  visualization_msgs::MarkerArray * marker_array,
  const std::optional<ros::Time> & current_time)
{
  for (const auto & marker : additional_marker_array.markers) {
    marker_array->markers.push_back(marker);
    if (current_time) marker_array->markers.back().header.stamp = current_time.value();
  }
}
}  // namespace autoware_utils_visualization
