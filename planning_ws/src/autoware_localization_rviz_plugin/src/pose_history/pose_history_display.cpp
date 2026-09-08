// Copyright 2020 Tier IV, Inc.
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

#include "pose_history_display.hpp"

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/validate_floats.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <memory>

namespace rviz_plugins
{
PoseHistory::PoseHistory() : last_stamp_(0)
{
  property_buffer_size_ = new rviz::IntProperty("Buffer Size", 100, "", this);
  property_line_view_ = new rviz::BoolProperty("Line", true, "", this);
  property_line_width_ =
    new rviz::FloatProperty("Width", 0.1, "", property_line_view_);
  property_line_alpha_ =
    new rviz::FloatProperty("Alpha", 1.0, "", property_line_view_);
  property_line_alpha_->setMin(0.0);
  property_line_alpha_->setMax(1.0);
  property_line_color_ =
    new rviz::ColorProperty("Color", Qt::white, "", property_line_view_);

  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(16000);
  property_line_width_->setMin(0.0);
}

PoseHistory::~PoseHistory() = default;  // Properties are deleted by Qt

void PoseHistory::onInitialize()
{
  MFDClass::onInitialize();
  lines_ = std::make_unique<rviz::BillboardLine>(scene_manager_, scene_node_);
}

// cppcheck-suppress unusedFunction
void PoseHistory::onEnable()
{
  subscribe();
}

// cppcheck-suppress unusedFunction
void PoseHistory::onDisable()
{
  unsubscribe();
}

void PoseHistory::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if (!history_.empty()) {
    lines_->clear();
    if (property_line_view_->getBool()) {
      update_lines();
    }
  }
}

void PoseHistory::subscribe()
{
  MFDClass::subscribe();
}

void PoseHistory::unsubscribe()
{
  MFDClass::unsubscribe();

  history_.clear();
  lines_->clear();
}

void PoseHistory::processMessage(const geometry_msgs::PoseStamped::ConstPtr & message)
{
  if (!rviz::validateFloats(message->pose)) {
    setStatus(
      rviz::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }
  if (target_frame_ != message->header.frame_id) {
    history_.clear();
    target_frame_ = message->header.frame_id;
  }
  history_.emplace_back(message);
  last_stamp_ = message->header.stamp;

  update_history();
}

void PoseHistory::update_history()
{
  const auto buffer_size = static_cast<size_t>(property_buffer_size_->getInt());
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

void PoseHistory::update_lines()
{
  Ogre::ColourValue color = rviz::qtToOgre(property_line_color_->getColor());
  color.a = property_line_alpha_->getFloat();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(target_frame_, last_stamp_, position, orientation)) {
    setStatus(rviz::StatusProperty::Error, "Transform", "Missing transform to fixed frame");
    return;
  }

  setStatus(rviz::StatusProperty::Ok, "Transform", "OK");
  lines_->setMaxPointsPerLine(history_.size());
  lines_->setLineWidth(property_line_width_->getFloat());
  lines_->setPosition(position);
  lines_->setOrientation(orientation);
  lines_->setColor(color.r, color.g, color.b, color.a);

  for (const auto & message : history_) {
    Ogre::Vector3 point;
    point.x = static_cast<float>(message->pose.position.x);
    point.y = static_cast<float>(message->pose.position.y);
    point.z = static_cast<float>(message->pose.position.z);
    lines_->addPoint(point);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseHistory, rviz::Display)
