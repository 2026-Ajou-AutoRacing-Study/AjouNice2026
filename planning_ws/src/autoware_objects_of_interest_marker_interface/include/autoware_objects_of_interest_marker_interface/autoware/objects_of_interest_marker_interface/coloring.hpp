// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__COLORING_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__COLORING_HPP_

#include "autoware/objects_of_interest_marker_interface/marker_data.hpp"

#include <autoware_utils_visualization/marker_helper.hpp>
#include <std_msgs/ColorRGBA.h>

namespace autoware::objects_of_interest_marker_interface::coloring
{
std_msgs::ColorRGBA getGreen(float alpha);
std_msgs::ColorRGBA getAmber(float alpha);
std_msgs::ColorRGBA getRed(float alpha);
std_msgs::ColorRGBA getGray(float alpha);
}  // namespace autoware::objects_of_interest_marker_interface::coloring

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__COLORING_HPP_
