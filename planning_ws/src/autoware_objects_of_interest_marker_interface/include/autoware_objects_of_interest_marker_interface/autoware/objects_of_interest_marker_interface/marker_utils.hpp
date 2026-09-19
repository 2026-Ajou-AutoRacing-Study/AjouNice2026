// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_UTILS_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_UTILS_HPP_

#include "autoware/objects_of_interest_marker_interface/coloring.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_data.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/constants.hpp>
#include <autoware_utils_math/trigonometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <string>

namespace autoware::objects_of_interest_marker_interface::marker_utils
{
visualization_msgs::Marker createArrowMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double height_offset,
  double arrow_length = 1.0);
visualization_msgs::Marker createCircleMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double radius,
  double height_offset, double line_width = 0.1);
visualization_msgs::Marker createNameTextMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double height_offset,
  double text_size = 0.5);
visualization_msgs::MarkerArray createTargetMarker(
  size_t id, const ObjectMarkerData & data, const std::string & name, double height_offset,
  double arrow_length = 1.0, double line_width = 0.1);
}  // namespace autoware::objects_of_interest_marker_interface::marker_utils

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_UTILS_HPP_
