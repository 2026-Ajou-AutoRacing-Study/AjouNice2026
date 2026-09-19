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

#ifndef SPEED_DISPLAY_HPP_
#define SPEED_DISPLAY_HPP_

#include <QImage>
#include <QString>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/display.h>

#include "autoware_vehicle_msgs/VelocityReport.h"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware_overlay_rviz_plugin
{

class SpeedDisplay
{
public:
  SpeedDisplay();
  void drawSpeedDisplay(QPainter & painter, const QRectF & backgroundRect, const QColor & color);
  void updateSpeedData(const autoware_vehicle_msgs::VelocityReport::ConstPtr & msg);

private:
  float current_speed_;  // Internal variable to store current speed
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // SPEED_DISPLAY_HPP_
