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

#ifndef GEAR_DISPLAY_HPP_
#define GEAR_DISPLAY_HPP_

#include <QImage>
#include <QString>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/display.h>

#include "autoware_vehicle_msgs/GearReport.h"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware_overlay_rviz_plugin
{

class GearDisplay
{
public:
  GearDisplay();
  void drawGearIndicator(
    QPainter & painter, const QRectF & backgroundRect, const QColor & color,
    const QColor & bg_color);
  void updateGearData(const autoware_vehicle_msgs::GearReport::ConstPtr & msg);

private:
  int current_gear_;  // Internal variable to store current gear
  QColor gray = QColor(194, 194, 194);
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // GEAR_DISPLAY_HPP_
