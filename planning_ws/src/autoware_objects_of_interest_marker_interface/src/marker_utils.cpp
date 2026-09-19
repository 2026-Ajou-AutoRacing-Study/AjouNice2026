// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware/objects_of_interest_marker_interface/marker_utils.hpp"

namespace autoware::objects_of_interest_marker_interface::marker_utils
{
using geometry_msgs::Point;
using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_scale;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

Marker createArrowMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double height_offset,
  double arrow_length)
{
  const double line_width = 0.25 * arrow_length;
  const double head_width = 0.5 * arrow_length;
  const double head_height = 0.5 * arrow_length;
  Marker marker = create_default_marker(
    "map", ros::Time::now(), name + "_arrow", static_cast<int32_t>(id), Marker::ARROW,
    create_marker_scale(line_width, head_width, head_height), data.color);
  const double height = 0.5 * data.shape.dimensions.z;
  Point src = data.pose.position;
  src.z += height + height_offset + arrow_length;
  Point dst = data.pose.position;
  dst.z += height + height_offset;
  marker.points.push_back(src);
  marker.points.push_back(dst);
  return marker;
}

Marker createCircleMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double radius,
  double height_offset, double line_width)
{
  Marker marker = create_default_marker(
    "map", ros::Time::now(), name, static_cast<int32_t>(id), Marker::LINE_STRIP,
    create_marker_scale(line_width, 0.0, 0.0), data.color);
  const double height = 0.5 * data.shape.dimensions.z;
  constexpr size_t num_points = 20;
  for (size_t i = 0; i < num_points; ++i) {
    Point point;
    const double theta = 2 * autoware_utils_math::pi * static_cast<double>(i) / num_points;
    point.x = data.pose.position.x + radius * autoware_utils_math::cos(static_cast<float>(theta));
    point.y = data.pose.position.y + radius * autoware_utils_math::sin(static_cast<float>(theta));
    point.z = data.pose.position.z + height + height_offset;
    marker.points.push_back(point);
  }
  marker.points.push_back(marker.points.front());
  return marker;
}

Marker createNameTextMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double height_offset,
  double text_size)
{
  Marker marker = create_default_marker(
    "map", ros::Time::now(), name + "_name_text", static_cast<int32_t>(id),
    Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, text_size),
    coloring::getGray(data.color.a));
  marker.text = name;
  marker.pose = data.pose;
  marker.pose.position.z += 0.5 * data.shape.dimensions.z + height_offset;
  return marker;
}

MarkerArray createTargetMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double height_offset,
  double arrow_length, double line_width)
{
  MarkerArray marker_array;
  marker_array.markers.push_back(createArrowMarker(id, data, name, height_offset, arrow_length));
  marker_array.markers.push_back(createCircleMarker(
    id, data, name + "_circle1", 0.5 * arrow_length, height_offset + 0.75 * arrow_length,
    line_width));
  marker_array.markers.push_back(createCircleMarker(
    id, data, name + "_circle2", 0.75 * arrow_length, height_offset + 0.75 * arrow_length,
    line_width));
  marker_array.markers.push_back(createNameTextMarker(
    id, data, name, height_offset + 1.5 * arrow_length, 0.5 * arrow_length));
  return marker_array;
}
}  // namespace autoware::objects_of_interest_marker_interface::marker_utils
