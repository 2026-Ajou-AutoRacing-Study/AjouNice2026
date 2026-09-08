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

#ifndef SIGNAL_DISPLAY_HPP_
#define SIGNAL_DISPLAY_HPP_
#ifndef Q_MOC_RUN
#include "gear_display.hpp"
#include "speed_display.hpp"
#include "speed_limit_display.hpp"
#include "steering_wheel_display.hpp"
#include "traffic_display.hpp"
#include "turn_signals_display.hpp"

#include <QImage>
#include <QString>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz/display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <memory>
#include <mutex>
#endif

namespace autoware_overlay_rviz_plugin
{
class SignalDisplay : public rviz::Display
{
  Q_OBJECT
public:
  SignalDisplay();
  ~SignalDisplay() override;

protected:
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateOverlaySize();
  void updateSmallOverlaySize();
  void updateOverlayPosition();
  void updateOverlayColor();
  void updateTurnSignalBlinkingMode();
  void topic_updated_gear();
  void topic_updated_steering();
  void topic_updated_speed();
  void topic_updated_speed_limit();
  void topic_updated_turn_signals();
  void topic_updated_hazard_lights();
  void topic_updated_traffic();

private:
  std::mutex mutex_;
  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
  rviz::IntProperty * property_width_;
  rviz::IntProperty * property_height_;
  rviz::IntProperty * property_left_;
  rviz::IntProperty * property_top_;
  rviz::ColorProperty * property_signal_color_;
  rviz::FloatProperty * property_handle_angle_scale_;
  rviz::ColorProperty * property_background_color_;
  rviz::FloatProperty * property_background_alpha_;
  rviz::ColorProperty * property_primary_color_;
  rviz::ColorProperty * property_light_limit_color_;
  rviz::ColorProperty * property_dark_limit_color_;

  rviz::EnumProperty * property_turn_signal_blinking_mode_;

  std::unique_ptr<rviz::RosTopicProperty> steering_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> gear_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> speed_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> turn_signals_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> hazard_lights_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> traffic_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> speed_limit_topic_property_;

  void drawHorizontalRoundedRectangle(QPainter & painter, const QRectF & backgroundRect);
  void drawVerticalRoundedRectangle(QPainter & painter, const QRectF & backgroundRect);
  void setupRosSubscriptions();

  std::unique_ptr<SteeringWheelDisplay> steering_wheel_display_;
  std::unique_ptr<GearDisplay> gear_display_;
  std::unique_ptr<SpeedDisplay> speed_display_;
  std::unique_ptr<TurnSignalsDisplay> turn_signals_display_;
  std::unique_ptr<TrafficDisplay> traffic_display_;
  std::unique_ptr<SpeedLimitDisplay> speed_limit_display_;

  ros::Subscriber gear_sub_, steering_sub_, speed_sub_, turn_signals_sub_, hazard_lights_sub_, traffic_sub_, speed_limit_sub_;

  std::mutex property_mutex_;

  void updateGearData(const autoware_vehicle_msgs::GearReport::ConstPtr & msg);
  void updateSteeringData(const autoware_vehicle_msgs::SteeringReport::ConstPtr & msg);
  void updateSpeedData(const autoware_vehicle_msgs::VelocityReport::ConstPtr & msg);
  void updateTurnSignalsData(
    const autoware_vehicle_msgs::TurnIndicatorsReport::ConstPtr & msg);
  void updateHazardLightsData(
    const autoware_vehicle_msgs::HazardLightsReport::ConstPtr & msg);
  void updateSpeedLimitData(
    const autoware_internal_planning_msgs::VelocityLimit::ConstPtr msg);
  void updateTrafficLightData(
    const autoware_perception_msgs::TrafficLightGroup::ConstPtr msg);
  void drawWidget(QImage & hud);
};
}  // namespace autoware_overlay_rviz_plugin

#endif  // SIGNAL_DISPLAY_HPP_
