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

#include "signal_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/ogre_helpers/render_system.h>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>

namespace autoware_overlay_rviz_plugin
{

SignalDisplay::SignalDisplay()
{
  property_width_ = new rviz::IntProperty(
    "Width", 550, "Width of the overlay", this, SLOT(updateOverlaySize()));
  property_height_ = new rviz::IntProperty(
    "Height", 100, "Height of the overlay", this, SLOT(updateOverlaySize()));
  property_left_ = new rviz::IntProperty(
    "Left", 0, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
  property_top_ = new rviz::IntProperty(
    "Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));
  property_signal_color_ = new rviz::ColorProperty(
    "Signal Color", QColor(QString("#00E678")), "Color of the signal arrows", this,
    SLOT(updateOverlayColor()));
  property_handle_angle_scale_ = new rviz::FloatProperty(
    "Handle Angle Scale", 17.0, "Scale of the steering wheel handle angle", this,
    SLOT(updateOverlaySize()));
  property_background_color_ = new rviz::ColorProperty(
    "Background Color", QColor(0, 0, 0), "Color of the signal arrows", this,
    SLOT(updateOverlayColor()));
  property_background_alpha_ = new rviz::FloatProperty(
    "Background Alpha", 0.3, "Background Color Alpha", this, SLOT(updateOverlayColor()));
  property_primary_color_ = new rviz::ColorProperty(
    "Primary Color", QColor(174, 174, 174), "Color of the signal arrows", this,
    SLOT(updateOverlayColor()));
  property_light_limit_color_ = new rviz::ColorProperty(
    "Light Traffic Color", QColor(255, 153, 153), "Color of the signal arrows", this,
    SLOT(updateOverlayColor()));
  property_dark_limit_color_ = new rviz::ColorProperty(
    "Dark Traffic Color", QColor(255, 51, 51), "Color of the signal arrows", this,
    SLOT(updateOverlayColor()));
  property_turn_signal_blinking_mode_ = new rviz::EnumProperty(
    "Signal Blinking Mode", "Static", "State of the signal blinking", this,
    SLOT(updateTurnSignalBlinkingMode()));
  property_turn_signal_blinking_mode_->addOption("Static", 0);
  property_turn_signal_blinking_mode_->addOption("Blinking", 1);

  // Initialize the component displays
  steering_wheel_display_ = std::make_unique<SteeringWheelDisplay>();
  gear_display_ = std::make_unique<GearDisplay>();
  speed_display_ = std::make_unique<SpeedDisplay>();
  turn_signals_display_ = std::make_unique<TurnSignalsDisplay>();
  traffic_display_ = std::make_unique<TrafficDisplay>();
  speed_limit_display_ = std::make_unique<SpeedLimitDisplay>();
}

void SignalDisplay::onInitialize()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  rviz::Display::onInitialize();
  rviz::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "SignalDisplayObject" << count++;
  overlay_.reset(new rviz_2d_overlay_plugins::OverlayObject(ss.str()));
  overlay_->show();
  updateOverlaySize();
  updateOverlayPosition();


  gear_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Gear Topic Test", "/vehicle/status/gear_status", "autoware_vehicle_msgs/msg/GearReport",
    "Topic for Gear Data", this, SLOT(topic_updated_gear()));

  turn_signals_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Turn Signals Topic", "/vehicle/status/turn_indicators_status",
    "autoware_vehicle_msgs/msg/TurnIndicatorsReport", "Topic for Turn Signals Data", this,
    SLOT(topic_updated_turn_signals()));

  speed_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Speed Topic", "/vehicle/status/velocity_status", "autoware_vehicle_msgs/msg/VelocityReport",
    "Topic for Speed Data", this, SLOT(topic_updated_speed()));

  steering_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Steering Topic", "/vehicle/status/steering_status", "autoware_vehicle_msgs/msg/SteeringReport",
    "Topic for Steering Data", this, SLOT(topic_updated_steering()));

  hazard_lights_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Hazard Lights Topic", "/vehicle/status/hazard_lights_status",
    "autoware_vehicle_msgs/msg/HazardLightsReport", "Topic for Hazard Lights Data", this,
    SLOT(topic_updated_hazard_lights()));

  speed_limit_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Speed Limit Topic", "/planning/scenario_planning/current_max_velocity",
    "autoware_internal_planning_msgs/msg/VelocityLimit", "Topic for Speed Limit Data", this,
    SLOT(topic_updated_speed_limit()));

  traffic_topic_property_ = std::make_unique<rviz::RosTopicProperty>(
    "Traffic Topic",
    "/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal",
    "autoware_perception_msgs/msgs/msg/TrafficLightGroup", "Topic for Traffic Light Data", this,
    SLOT(topic_updated_traffic()));
}

void SignalDisplay::setupRosSubscriptions()
{
  topic_updated_gear();
  topic_updated_steering();
  topic_updated_speed();
  topic_updated_speed_limit();
  topic_updated_turn_signals();
  topic_updated_hazard_lights();
  topic_updated_traffic();
}

SignalDisplay::~SignalDisplay()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  overlay_.reset();

  gear_sub_.shutdown();
  steering_sub_.shutdown();
  speed_sub_.shutdown();
  turn_signals_sub_.shutdown();
  hazard_lights_sub_.shutdown();
  traffic_sub_.shutdown();

  steering_wheel_display_.reset();
  gear_display_.reset();
  speed_display_.reset();
  turn_signals_display_.reset();
  traffic_display_.reset();
  speed_limit_display_.reset();

  gear_topic_property_.reset();
  turn_signals_topic_property_.reset();
  speed_topic_property_.reset();
  steering_topic_property_.reset();
  hazard_lights_topic_property_.reset();
  traffic_topic_property_.reset();
}

void SignalDisplay::update(float /* wall_dt */, float /* ros_dt */)
{
  if (!overlay_) {
    return;
  }
  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(Qt::transparent);
  drawWidget(hud);
}

void SignalDisplay::onEnable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  if (overlay_) {
    overlay_->show();
  }
  setupRosSubscriptions();
}

void SignalDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  gear_sub_.shutdown();
  steering_sub_.shutdown();
  speed_sub_.shutdown();
  turn_signals_sub_.shutdown();
  hazard_lights_sub_.shutdown();

  if (overlay_) {
    overlay_->hide();
  }
}

void SignalDisplay::updateTrafficLightData(
  const autoware_perception_msgs::TrafficLightGroup::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (traffic_display_) {
    traffic_display_->updateTrafficLightData(msg);
  }
}

void SignalDisplay::updateSpeedLimitData(
  const autoware_internal_planning_msgs::VelocityLimit::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (speed_limit_display_) {
    speed_limit_display_->updateSpeedLimitData(msg);
    queueRender();
  }
}

void SignalDisplay::updateHazardLightsData(
  const autoware_vehicle_msgs::HazardLightsReport::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (turn_signals_display_) {
    turn_signals_display_->updateHazardLightsData(msg);
    queueRender();
  }
}

void SignalDisplay::updateGearData(
  const autoware_vehicle_msgs::GearReport::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (gear_display_) {
    gear_display_->updateGearData(msg);
    queueRender();
  }
}

void SignalDisplay::updateSteeringData(
  const autoware_vehicle_msgs::SteeringReport::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (steering_wheel_display_) {
    steering_wheel_display_->updateSteeringData(msg);
    queueRender();
  }
}

void SignalDisplay::updateSpeedData(
  const autoware_vehicle_msgs::VelocityReport::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (speed_display_) {
    speed_display_->updateSpeedData(msg);
    speed_limit_display_->updateSpeedData(msg);
    queueRender();
  }
}

void SignalDisplay::updateTurnSignalsData(
  const autoware_vehicle_msgs::TurnIndicatorsReport::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (turn_signals_display_) {
    turn_signals_display_->updateTurnSignalsData(msg);
    queueRender();
  }
}

void SignalDisplay::drawWidget(QImage & hud)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (!overlay_->isVisible()) {
    return;
  }

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QRectF backgroundRect(0, 0, hud.width(), hud.height());
  drawHorizontalRoundedRectangle(painter, backgroundRect);

  // Draw components
  if (gear_display_) {
    gear_display_->drawGearIndicator(
      painter, backgroundRect, property_primary_color_->getColor(),
      property_background_color_->getColor());
  }

  if (steering_wheel_display_) {
    steering_wheel_display_->drawSteeringWheel(
      painter, backgroundRect, property_handle_angle_scale_->getFloat());
  }

  if (speed_display_) {
    speed_display_->drawSpeedDisplay(painter, backgroundRect, property_primary_color_->getColor());
  }
  if (turn_signals_display_) {
    turn_signals_display_->drawArrows(painter, backgroundRect, property_signal_color_->getColor());
  }

  if (traffic_display_) {
    traffic_display_->drawTrafficLightIndicator(painter, backgroundRect);
  }

  if (speed_limit_display_) {
    speed_limit_display_->drawSpeedLimitIndicator(
      painter, backgroundRect, property_primary_color_->getColor(),
      property_light_limit_color_->getColor(), property_dark_limit_color_->getColor(),
      property_background_color_->getColor(), property_background_alpha_->getFloat());
  }

  painter.end();
}

void SignalDisplay::drawHorizontalRoundedRectangle(
  QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(
    property_background_color_->getColor().hue(),
    property_background_color_->getColor().saturation(),
    property_background_color_->getColor().value());
  colorFromHSV.setAlphaF(property_background_alpha_->getFloat());
  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    backgroundRect, backgroundRect.height() / 2, backgroundRect.height() / 2);  // Circular ends
}
void SignalDisplay::drawVerticalRoundedRectangle(QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(
    property_background_color_->getColor().hue(),
    property_background_color_->getColor().saturation(),
    property_background_color_->getColor().value());
  colorFromHSV.setAlphaF(property_background_alpha_->getFloat());

  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    backgroundRect, backgroundRect.width() / 2, backgroundRect.width() / 2);  // Circular ends
}

void SignalDisplay::reset()
{
  rviz::Display::reset();
  overlay_->hide();
}

void SignalDisplay::updateOverlaySize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  queueRender();
}

void SignalDisplay::updateOverlayPosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->setPosition(
    property_left_->getInt(), property_top_->getInt(),
    rviz_2d_overlay_plugins::HorizontalAlignment::CENTER,
    rviz_2d_overlay_plugins::VerticalAlignment::TOP);
  queueRender();
}

void SignalDisplay::updateOverlayColor()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queueRender();
}

void SignalDisplay::updateTurnSignalBlinkingMode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (turn_signals_display_) {
    turn_signals_display_->setBlinkingMode(property_turn_signal_blinking_mode_->getStdString());
  }
  queueRender();
}

void SignalDisplay::topic_updated_gear()
{
  gear_sub_ = update_nh_.subscribe(gear_topic_property_->getTopicStd(), 10, &SignalDisplay::updateGearData, this);
}

void SignalDisplay::topic_updated_steering()
{
  // resubscribe to the topic
  steering_sub_ = update_nh_.subscribe(steering_topic_property_->getTopicStd(), 10, &SignalDisplay::updateSteeringData, this);
}

void SignalDisplay::topic_updated_speed()
{
  // resubscribe to the topic
  speed_sub_ = update_nh_.subscribe(speed_topic_property_->getTopicStd(), 10, &SignalDisplay::updateSpeedData, this);
}

void SignalDisplay::topic_updated_speed_limit()
{
  // resubscribe to the topic
  speed_limit_sub_ = update_nh_.subscribe(speed_limit_topic_property_->getTopicStd(), 10, &SignalDisplay::updateSpeedLimitData, this);
}

void SignalDisplay::topic_updated_turn_signals()
{
  // resubscribe to the topic
  turn_signals_sub_ = update_nh_.subscribe(turn_signals_topic_property_->getTopicStd(), 10, &SignalDisplay::updateTurnSignalsData, this);
}

void SignalDisplay::topic_updated_hazard_lights()
{
  // resubscribe to the topic
  hazard_lights_sub_ = update_nh_.subscribe(hazard_lights_topic_property_->getTopicStd(), 10, &SignalDisplay::updateHazardLightsData, this);
}

void SignalDisplay::topic_updated_traffic()
{
  // resubscribe to the topic
  traffic_sub_ = update_nh_.subscribe(traffic_topic_property_->getTopicStd(), 10, &SignalDisplay::updateTrafficLightData, this);
}

}  // namespace autoware_overlay_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_overlay_rviz_plugin::SignalDisplay, rviz::Display)
