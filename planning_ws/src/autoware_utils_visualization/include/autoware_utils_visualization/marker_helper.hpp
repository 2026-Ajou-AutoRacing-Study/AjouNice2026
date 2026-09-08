// Copyright 2020 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE_UTILS_VISUALIZATION__MARKER_HELPER_HPP_
#define AUTOWARE_UTILS_VISUALIZATION__MARKER_HELPER_HPP_

#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

#include <optional>
#include <string>

namespace autoware_utils_visualization
{
inline constexpr double MARKER_LIFETIME = 0.5;

inline geometry_msgs::Point create_marker_position(double x, double y, double z)
{
  geometry_msgs::Point point;
  point.x = x; point.y = y; point.z = z;
  return point;
}

inline geometry_msgs::Quaternion create_marker_orientation(double x, double y, double z, double w)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.x = x; quaternion.y = y; quaternion.z = z; quaternion.w = w;
  return quaternion;
}

inline geometry_msgs::Vector3 create_marker_scale(double x, double y, double z)
{
  geometry_msgs::Vector3 scale;
  scale.x = x; scale.y = y; scale.z = z;
  return scale;
}

inline std_msgs::ColorRGBA create_marker_color(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;
  color.r = r; color.g = g; color.b = b; color.a = a;
  return color;
}

visualization_msgs::Marker create_default_marker(
  const std::string & frame_id, const ros::Time & now, const std::string & ns, int32_t id,
  int32_t type, const geometry_msgs::Vector3 & scale, const std_msgs::ColorRGBA & color);

visualization_msgs::Marker create_deleted_default_marker(
  const ros::Time & now, const std::string & ns, int32_t id);

void append_marker_array(
  const visualization_msgs::MarkerArray & additional_marker_array,
  visualization_msgs::MarkerArray * marker_array,
  const std::optional<ros::Time> & current_time = {});
}  // namespace autoware_utils_visualization

#endif  // AUTOWARE_UTILS_VISUALIZATION__MARKER_HELPER_HPP_
