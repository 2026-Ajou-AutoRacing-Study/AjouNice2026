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

#ifndef TOOLS__MAX_VELOCITY_HPP_
#define TOOLS__MAX_VELOCITY_HPP_

#include <deque>
#include <memory>

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/tool.h>
#include <rviz/validate_floats.h>

#include <autoware_internal_planning_msgs/VelocityLimit.h>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#endif

namespace rviz_plugins
{
class MaxVelocityDisplay : public rviz::Display
{
  Q_OBJECT

public:
  MaxVelocityDisplay();
  virtual ~MaxVelocityDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();

private Q_SLOTS:
  void updateTopic();
  void updateVisualization();

protected:
  void processMessage(
    const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & msg_ptr);
  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
  rviz::ColorProperty * property_text_color_;
  rviz::IntProperty * property_left_;
  rviz::IntProperty * property_top_;
  rviz::IntProperty * property_length_;
  rviz::StringProperty * property_topic_name_;
  rviz::FloatProperty * property_value_scale_;

private:
  ros::Subscriber max_vel_sub_;
  autoware_internal_planning_msgs::VelocityLimit::ConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__MAX_VELOCITY_HPP_
