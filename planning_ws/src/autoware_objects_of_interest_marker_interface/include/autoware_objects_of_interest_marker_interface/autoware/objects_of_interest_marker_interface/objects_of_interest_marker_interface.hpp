// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_

#include "autoware/objects_of_interest_marker_interface/coloring.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_data.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_utils.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace autoware::objects_of_interest_marker_interface
{
class ObjectsOfInterestMarkerInterface
{
public:
  ObjectsOfInterestMarkerInterface(ros::NodeHandle * node, const std::string & name);
  void insertObjectData(
    const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
    const ColorName & color_name);
  void insertObjectDataWithCustomColor(
    const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
    const std_msgs::ColorRGBA & color);
  void publishMarkerArray();
  void setHeightOffset(double offset);
  static std_msgs::ColorRGBA getColor(const ColorName & color_name, float alpha = 0.99f);
  std::string getName() const { return name_; }
  double getHeightOffset() const { return height_offset_; }

private:
  ros::Publisher pub_marker_;
  double height_offset_{0.5};
  std::vector<ObjectMarkerData> obj_marker_data_array_;
  std::string name_;
  std::string topic_namespace_ = "/planning/debug/objects_of_interest";
};
}  // namespace autoware::objects_of_interest_marker_interface

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
