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

#ifndef CONTROL_MODE_DISPLAY_HPP_
#define CONTROL_MODE_DISPLAY_HPP_

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <autoware_vehicle_msgs/ControlModeReport.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <memory>
#include <string>

namespace rviz_plugins
{

class ControlModeDisplay : public rviz::Panel
{
  Q_OBJECT

public:
  explicit ControlModeDisplay(QWidget * parent = nullptr);
  ~ControlModeDisplay() override;

protected:
  void onInitialize() override;
  void load(const rviz::Config & config) override;
  void save(rviz::Config config) const override;

private:
  void processMessage(const autoware_vehicle_msgs::ControlModeReport::ConstPtr & msg);
  std::string getModeString(uint8_t mode) const;
  void subscribe();
  void unsubscribe();

  ros::NodeHandle nh_;
  ros::Subscriber subscription_;
  autoware_vehicle_msgs::ControlModeReport::ConstPtr last_msg_;
  QLabel * mode_label_;
};

}  // namespace rviz_plugins

#endif  // CONTROL_MODE_DISPLAY_HPP_
