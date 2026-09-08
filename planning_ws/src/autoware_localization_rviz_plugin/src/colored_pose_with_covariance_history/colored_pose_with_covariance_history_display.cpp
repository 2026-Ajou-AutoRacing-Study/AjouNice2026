// Copyright 2025 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "colored_pose_with_covariance_history_display.hpp"

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/validate_floats.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <ros/message_traits.h>

#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <memory>

namespace rviz_plugins
{
ColoredPoseWithCovarianceHistory::ColoredPoseWithCovarianceHistory()
: last_stamp_(0)
{
  property_value_type_ = new rviz::EnumProperty(
    "Value Type", "Float32", "Select the type of value to visualize", this,
    SLOT(update_value_type()));
  property_value_type_->addOption("Int32", static_cast<int>(ValueType::Int32));
  property_value_type_->addOption("Float32", static_cast<int>(ValueType::Float32));

  property_value_topic_ = new rviz::RosTopicProperty(
    "Value Topic", "",
    ros::message_traits::datatype<autoware_internal_debug_msgs::Float32Stamped>(), "", this,
    SLOT(update_value_topic()));

  property_buffer_size_ = new rviz::IntProperty("Buffer Size", 100, "", this);
  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(100000);

  property_line_view_ = new rviz::BoolProperty("Path", true, "", this);

  property_line_width_ =
    new rviz::FloatProperty("Width", 0.1, "", property_line_view_);
  property_line_width_->setMin(0.0);

  property_line_alpha_ =
    new rviz::FloatProperty("Alpha", 1.0, "", property_line_view_);
  property_line_alpha_->setMin(0.0);
  property_line_alpha_->setMax(1.0);

  property_line_min_color_ =
    new rviz::ColorProperty("Min Color", Qt::blue, "", property_line_view_);

  property_line_max_color_ =
    new rviz::ColorProperty("Max Color", Qt::red, "", property_line_view_);

  property_line_less_color_ =
    new rviz::ColorProperty("Less Color", Qt::blue, "", property_line_view_);

  property_line_greater_color_ =
    new rviz::ColorProperty("Greater Color", Qt::red, "", property_line_view_);

  property_auto_min_max_ =
    new rviz::BoolProperty("Auto Set Min/Max", true, "", this);

  property_min_value_ =
    new rviz::FloatProperty("Min Value", 0.0, "", property_auto_min_max_);

  property_max_value_ =
    new rviz::FloatProperty("Max Value", 1.0, "", property_auto_min_max_);
}

ColoredPoseWithCovarianceHistory::~ColoredPoseWithCovarianceHistory()
{
  delete property_value_type_;
}

void ColoredPoseWithCovarianceHistory::onInitialize()
{
  MFDClass::onInitialize();
  lines_ = std::make_unique<rviz::BillboardLine>(scene_manager_, scene_node_);
  update_value_type();
}

void ColoredPoseWithCovarianceHistory::onEnable()
{
  subscribe();
}

void ColoredPoseWithCovarianceHistory::onDisable()
{
  unsubscribe();
}

void ColoredPoseWithCovarianceHistory::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if (!history_.empty()) {
    lines_->clear();
    if (property_line_view_->getBool()) {
      update_visualization();
    }
  }
}

void ColoredPoseWithCovarianceHistory::update_value_type()
{
  current_value_type_ = static_cast<ValueType>(property_value_type_->getOptionInt());

  if (current_value_type_ == ValueType::Int32) {
    property_value_topic_->setMessageType(
      ros::message_traits::datatype<autoware_internal_debug_msgs::Int32Stamped>());
  } else {
    property_value_topic_->setMessageType(
      ros::message_traits::datatype<autoware_internal_debug_msgs::Float32Stamped>());
  }

  update_value_topic();
}

void ColoredPoseWithCovarianceHistory::update_value_topic()
{
  lines_->clear();
  int32_sub_.shutdown();
  float32_sub_.shutdown();
  if (property_value_topic_->getStdString().empty()) {
    setStatus(rviz::StatusProperty::Error, "Value Topic", "No topic was set.");
    return;
  }
  if (current_value_type_ == ValueType::Int32) {
    int32_sub_ = update_nh_.subscribe<autoware_internal_debug_msgs::Int32Stamped>(
      property_value_topic_->getStdString(), 10,
      &ColoredPoseWithCovarianceHistory::process_int32_message, this);
  } else {
    float32_sub_ = update_nh_.subscribe<autoware_internal_debug_msgs::Float32Stamped>(
      property_value_topic_->getStdString(), 10,
      &ColoredPoseWithCovarianceHistory::process_float32_message, this);
  }
  setStatus(rviz::StatusProperty::Ok, "Value Topic", "OK");
}

void ColoredPoseWithCovarianceHistory::update_visualization()
{
  if (history_.empty()) {
    setStatus(rviz::StatusProperty::Error, "Topic", "No messages received yet");
    return;
  }
  if (property_auto_min_max_->getBool()) {
    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::min();
    for (const auto & pose_with_value : history_) {
      if (pose_with_value.value < min_value) {
        min_value = pose_with_value.value;
      }
      if (pose_with_value.value > max_value) {
        max_value = pose_with_value.value;
      }
    }
    property_min_value_->setFloat(min_value);
    property_max_value_->setFloat(max_value);
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(target_frame_, last_stamp_, position, orientation)) {
    setStatus(rviz::StatusProperty::Error, "Transform", "Missing transform to fixed frame");
    return;
  }

  setStatus(rviz::StatusProperty::Ok, "Transform", "OK");

  lines_->clear();
  lines_->setMaxPointsPerLine(history_.size());
  lines_->setLineWidth(property_line_width_->getFloat());
  lines_->setPosition(position);
  lines_->setOrientation(orientation);

  for (const auto & pose_with_value : history_) {
    const auto color = get_color_from_value(pose_with_value.value);
    Ogre::Vector3 point;
    point.x = static_cast<float>(pose_with_value.pose->pose.pose.position.x);
    point.y = static_cast<float>(pose_with_value.pose->pose.pose.position.y);
    point.z = static_cast<float>(pose_with_value.pose->pose.pose.position.z);
    lines_->addPoint(point, color);
  }
}

void ColoredPoseWithCovarianceHistory::subscribe()
{
  history_.clear();
  lines_->clear();

  MFDClass::subscribe();
  update_value_topic();
}

void ColoredPoseWithCovarianceHistory::unsubscribe()
{
  MFDClass::unsubscribe();
  int32_sub_.shutdown();
  float32_sub_.shutdown();

  history_.clear();
  lines_->clear();
}

void ColoredPoseWithCovarianceHistory::processMessage(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & message)
{
  if (
    !rviz::validateFloats(message->pose.pose) ||
    !rviz::validateFloats(message->pose.covariance)) {
    setStatus(
      rviz::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }
  if (target_frame_ != message->header.frame_id) {
    history_.clear();
    target_frame_ = message->header.frame_id;
  }
  history_.emplace_back(message, last_value_);
  last_stamp_ = message->header.stamp;
  update_history();
  update_visualization();
}

void ColoredPoseWithCovarianceHistory::process_int32_message(
  const autoware_internal_debug_msgs::Int32Stamped::ConstPtr & message)
{
  last_value_ = static_cast<double>(message->data);
}

void ColoredPoseWithCovarianceHistory::process_float32_message(
  const autoware_internal_debug_msgs::Float32Stamped::ConstPtr & message)
{
  last_value_ = static_cast<double>(message->data);
}

void ColoredPoseWithCovarianceHistory::update_history()
{
  const auto buffer_size = static_cast<size_t>(property_buffer_size_->getInt());
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

Ogre::ColourValue ColoredPoseWithCovarianceHistory::get_color_from_value(double value)
{
  const auto min_value = property_min_value_->getFloat();
  const auto max_value = property_max_value_->getFloat();
  if (value < min_value) {
    return property_line_less_color_->getOgreColor();
  }
  if (value > max_value) {
    return property_line_greater_color_->getOgreColor();
  }
  if (min_value == max_value) {
    return property_line_min_color_->getOgreColor();  // Avoid division by zero
  }
  const auto ratio = (value - min_value) / (max_value - min_value);
  const auto min_color = property_line_min_color_->getOgreColor();
  const auto max_color = property_line_max_color_->getOgreColor();
  const auto color =
    min_color * (1.0f - static_cast<float>(ratio)) + max_color * static_cast<float>(ratio);
  return color;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ColoredPoseWithCovarianceHistory, rviz::Display)
