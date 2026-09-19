// Copyright 2025 TIER IV, Inc.
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

#include "control_mode_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <QMetaObject>
#include <QThread>

#include <string>

namespace rviz_plugins
{

ControlModeDisplay::ControlModeDisplay(QWidget * parent)
: rviz::Panel(parent), last_msg_(nullptr)
{
  auto * layout = new QVBoxLayout(this);

  mode_label_ = new QLabel(this);
  mode_label_->setAlignment(Qt::AlignCenter);
  mode_label_->setStyleSheet(
    "QLabel {"
    "  padding: 10px;"
    "  border-radius: 5px;"
    "  background-color: #333333;"
    "  color: white;"
    "  font-size: 14px;"
    "  font-weight: bold;"
    "}");

  layout->addWidget(mode_label_);
  setLayout(layout);
}

ControlModeDisplay::~ControlModeDisplay()
{
  unsubscribe();
}

void ControlModeDisplay::onInitialize()
{
  subscribe();
}

void ControlModeDisplay::load(const rviz::Config & config)
{
  rviz::Panel::load(config);
}

void ControlModeDisplay::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void ControlModeDisplay::processMessage(
  const autoware_vehicle_msgs::ControlModeReport::ConstPtr & msg)
{
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(this, [this, msg]() { processMessage(msg); }, Qt::QueuedConnection);
    return;
  }
  last_msg_ = msg;
  std::string mode_str = getModeString(msg->mode);
  mode_label_->setText(QString::fromStdString(mode_str));

  QString bg_color;
  switch (msg->mode) {
    case 1:                  // AUTONOMOUS
      bg_color = "#4CAF50";  // Green
      break;
    case 4:                  // MANUAL
      bg_color = "#F44336";  // Red
      break;
    case 5:                  // DISENGAGED
      bg_color = "#FF9800";  // Orange
      break;
    default:
      bg_color = "#333333";  // Dark gray
      break;
  }

  mode_label_->setStyleSheet(QString(
                               "QLabel {"
                               "  padding: 10px;"
                               "  border-radius: 5px;"
                               "  background-color: %1;"
                               "  color: white;"
                               "  font-size: 14px;"
                               "  font-weight: bold;"
                               "}")
                               .arg(bg_color));
}

std::string ControlModeDisplay::getModeString(uint8_t mode) const
{
  switch (mode) {
    case 0:
      return "NO_COMMAND";
    case 1:
      return "AUTONOMOUS";
    case 2:
      return "AUTONOMOUS_STEER_ONLY";
    case 3:
      return "AUTONOMOUS_VELOCITY_ONLY";
    case 4:
      return "MANUAL";
    case 5:
      return "DISENGAGED";
    case 6:
      return "NOT_READY";
    default:
      return "UNKNOWN";
  }
}

void ControlModeDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  try {
    subscription_ = nh_.subscribe<autoware_vehicle_msgs::ControlModeReport>(
      "/vehicle/status/control_mode", 10, &ControlModeDisplay::processMessage, this);
  } catch (const std::exception & e) {
    mode_label_->setText("Error: Topic subscription failed");
    mode_label_->setStyleSheet(
      "QLabel {"
      "  padding: 10px;"
      "  border-radius: 5px;"
      "  background-color: #F44336;"
      "  color: white;"
      "  font-size: 14px;"
      "  font-weight: bold;"
      "}");
  }
}

void ControlModeDisplay::unsubscribe()
{
  subscription_.shutdown();
}

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::ControlModeDisplay, rviz::Panel)
