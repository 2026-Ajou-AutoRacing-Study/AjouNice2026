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

#ifndef POSE_HISTORY__POSE_HISTORY_DISPLAY_HPP_
#define POSE_HISTORY__POSE_HISTORY_DISPLAY_HPP_

#include <rviz/message_filter_display.h>

#include <geometry_msgs/PoseStamped.h>

#include <deque>
#include <memory>
#include <string>

namespace rviz
{
class BillboardLine;
}  // namespace rviz
namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
}  // namespace rviz

namespace rviz_plugins
{
class PoseHistory : public rviz::MessageFilterDisplay<geometry_msgs::PoseStamped>
{
  Q_OBJECT

public:
  PoseHistory();
  ~PoseHistory() override;
  PoseHistory(const PoseHistory &) = delete;
  PoseHistory(const PoseHistory &&) = delete;
  PoseHistory & operator=(const PoseHistory &) = delete;
  PoseHistory & operator=(const PoseHistory &&) = delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private:
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(const geometry_msgs::PoseStamped::ConstPtr & message) override;
  void update_history();
  void update_lines();

  std::string target_frame_;
  std::deque<geometry_msgs::PoseStamped::ConstPtr> history_;
  std::unique_ptr<rviz::BillboardLine> lines_;
  ros::Time last_stamp_;

  rviz::IntProperty * property_buffer_size_;
  rviz::BoolProperty * property_line_view_;
  rviz::FloatProperty * property_line_width_;
  rviz::FloatProperty * property_line_alpha_;
  rviz::ColorProperty * property_line_color_;
};

}  // namespace rviz_plugins

#endif  // POSE_HISTORY__POSE_HISTORY_DISPLAY_HPP_
