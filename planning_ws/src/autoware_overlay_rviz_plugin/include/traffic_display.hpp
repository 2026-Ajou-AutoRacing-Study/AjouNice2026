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

#ifndef TRAFFIC_DISPLAY_HPP_
#define TRAFFIC_DISPLAY_HPP_

#include <QImage>
#include <QString>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/display.h>

#include <autoware_perception_msgs/TrafficLightElement.h>
#include <autoware_perception_msgs/TrafficLightGroup.h>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware_overlay_rviz_plugin
{

class TrafficDisplay
{
public:
  TrafficDisplay();
  void drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect);
  void updateTrafficLightData(
    const autoware_perception_msgs::TrafficLightGroup::ConstPtr & msg);
  autoware_perception_msgs::TrafficLightGroup current_traffic_;

private:
  QImage traffic_light_image_;

  const QColor tl_red_;
  const QColor tl_yellow_;
  const QColor tl_green_;
  const QColor tl_gray_;

  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // TRAFFIC_DISPLAY_HPP_
