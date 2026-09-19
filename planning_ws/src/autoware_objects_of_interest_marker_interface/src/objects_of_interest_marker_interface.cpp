// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"

namespace autoware::objects_of_interest_marker_interface
{
ObjectsOfInterestMarkerInterface::ObjectsOfInterestMarkerInterface(
  ros::NodeHandle * node, const std::string & name)
: name_{name}
{
  pub_marker_ = node->advertise<visualization_msgs::MarkerArray>(topic_namespace_ + "/" + name, 1);
}

void ObjectsOfInterestMarkerInterface::insertObjectData(
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
  const ColorName & color_name)
{
  insertObjectDataWithCustomColor(pose, shape, getColor(color_name));
}

void ObjectsOfInterestMarkerInterface::insertObjectDataWithCustomColor(
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
  const std_msgs::ColorRGBA & color)
{
  ObjectMarkerData data;
  data.pose = pose;
  data.shape = shape;
  data.color = color;
  obj_marker_data_array_.push_back(data);
}

void ObjectsOfInterestMarkerInterface::publishMarkerArray()
{
  if (pub_marker_.getNumSubscribers() == 0) {
    return;
  }
  visualization_msgs::MarkerArray marker_array;
  for (size_t i = 0; i < obj_marker_data_array_.size(); ++i) {
    const auto data = obj_marker_data_array_.at(i);
    const auto target_marker = marker_utils::createTargetMarker(i, data, getName(), getHeightOffset());
    marker_array.markers.insert(
      marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
  }
  pub_marker_.publish(marker_array);
  obj_marker_data_array_.clear();
}

void ObjectsOfInterestMarkerInterface::setHeightOffset(double offset) { height_offset_ = offset; }

std_msgs::ColorRGBA ObjectsOfInterestMarkerInterface::getColor(
  const ColorName & color_name, float alpha)
{
  switch (color_name) {
    case ColorName::GRAY: return coloring::getGray(alpha);
    case ColorName::GREEN: return coloring::getGreen(alpha);
    case ColorName::AMBER: return coloring::getAmber(alpha);
    case ColorName::RED: return coloring::getRed(alpha);
    default: return coloring::getGray(alpha);
  }
}
}  // namespace autoware::objects_of_interest_marker_interface
