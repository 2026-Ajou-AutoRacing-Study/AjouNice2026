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

#ifndef TOOLS__STEERING_ANGLE_HPP_
#define TOOLS__STEERING_ANGLE_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include "unfiltered_topic_display.hpp"

#include <autoware_vehicle_msgs/SteeringReport.h>
#endif

namespace rviz_plugins
{
class SteeringAngleDisplay
: public UnfilteredTopicDisplay<autoware_vehicle_msgs::SteeringReport>
{
  Q_OBJECT

  using RTDClass = UnfilteredTopicDisplay<autoware_vehicle_msgs::SteeringReport>;

public:
  SteeringAngleDisplay();
  ~SteeringAngleDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const autoware_vehicle_msgs::SteeringReport::ConstPtr msg_ptr) override;

  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::ColorProperty * property_text_color_;
  rviz::IntProperty * property_left_;
  rviz::IntProperty * property_top_;
  rviz::IntProperty * property_length_;
  rviz::FloatProperty * property_handle_angle_scale_;
  rviz::IntProperty * property_value_height_offset_;
  rviz::FloatProperty * property_value_scale_;
  QPixmap handle_image_;
  // QImage hud_;

private:
  std::mutex mutex_;
  autoware_vehicle_msgs::SteeringReport::ConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__STEERING_ANGLE_HPP_
