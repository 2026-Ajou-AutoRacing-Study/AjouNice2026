// Copyright 2024 The Autoware Contributors
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

#ifndef SPEED_LIMIT_DISPLAY_HPP_
#define SPEED_LIMIT_DISPLAY_HPP_

#include <QImage>
#include <QString>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/display.h>

#include "autoware_vehicle_msgs/VelocityReport.h"
#include <autoware_internal_planning_msgs/VelocityLimit.h>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware_overlay_rviz_plugin
{

class SpeedLimitDisplay
{
public:
  SpeedLimitDisplay();
  void drawSpeedLimitIndicator(
    QPainter & painter, const QRectF & backgroundRect, const QColor & color,
    const QColor & light_color, const QColor & dark_color, const QColor & bg_color,
    const float bg_alpha);
  void updateSpeedLimitData(
    const autoware_internal_planning_msgs::VelocityLimit::ConstPtr msg);
  void updateSpeedData(const autoware_vehicle_msgs::VelocityReport::ConstPtr & msg);

private:
  float current_limit;   // Internal variable to store current gear
  float current_speed_;  // Internal variable to store current speed
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // SPEED_LIMIT_DISPLAY_HPP_
