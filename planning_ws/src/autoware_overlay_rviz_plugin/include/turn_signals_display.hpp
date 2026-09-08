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

#ifndef TURN_SIGNALS_DISPLAY_HPP_
#define TURN_SIGNALS_DISPLAY_HPP_

#include <QImage>
#include <QString>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/display.h>

#include <autoware_vehicle_msgs/HazardLightsReport.h>
#include <autoware_vehicle_msgs/TurnIndicatorsReport.h>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <string>
#include <string_view>

namespace autoware_overlay_rviz_plugin
{

class TurnSignalsDisplay
{
public:
  TurnSignalsDisplay();
  void drawArrows(QPainter & painter, const QRectF & backgroundRect, const QColor & color);
  void updateTurnSignalsData(
    const autoware_vehicle_msgs::TurnIndicatorsReport::ConstPtr & msg);
  void updateHazardLightsData(
    const autoware_vehicle_msgs::HazardLightsReport::ConstPtr & msg);
  void setBlinkingMode(std::string_view mode);

private:
  QImage arrowImage;
  QColor gray = QColor(79, 79, 79);

  std::string blinking_mode_ = "Static";

  int current_turn_signal_;    // Internal variable to store turn signal state
  int current_hazard_lights_;  // Internal variable to store hazard lights state
  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // TURN_SIGNALS_DISPLAY_HPP_
