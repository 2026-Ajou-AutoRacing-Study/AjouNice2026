// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_DATA_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_DATA_HPP_

#include <autoware_perception_msgs/PredictedObject.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

namespace autoware::objects_of_interest_marker_interface
{
struct ObjectMarkerData
{
  geometry_msgs::Pose pose{};
  autoware_perception_msgs::Shape shape{};
  std_msgs::ColorRGBA color;
};

enum class ColorName { GRAY, GREEN, AMBER, RED };
}  // namespace autoware::objects_of_interest_marker_interface

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_DATA_HPP_
