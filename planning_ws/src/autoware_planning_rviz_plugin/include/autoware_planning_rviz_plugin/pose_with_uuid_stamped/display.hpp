// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__POSE_WITH_UUID_STAMPED__DISPLAY_HPP_
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__POSE_WITH_UUID_STAMPED__DISPLAY_HPP_

#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>

#include <autoware_planning_msgs/PoseWithUuidStamped.h>

#include <deque>
#include <memory>
#include <string>

namespace rviz
{
class Axes;
class MovableText;
}  // namespace rviz
namespace rviz_plugins
{
class AutowarePoseWithUuidStampedDisplay
: public rviz::MessageFilterDisplay<autoware_planning_msgs::PoseWithUuidStamped>
{
  Q_OBJECT

public:
  AutowarePoseWithUuidStampedDisplay();
  ~AutowarePoseWithUuidStampedDisplay() override;
  AutowarePoseWithUuidStampedDisplay(const AutowarePoseWithUuidStampedDisplay &) = delete;
  AutowarePoseWithUuidStampedDisplay(const AutowarePoseWithUuidStampedDisplay &&) = delete;
  AutowarePoseWithUuidStampedDisplay & operator=(const AutowarePoseWithUuidStampedDisplay &) =
    delete;
  AutowarePoseWithUuidStampedDisplay & operator=(const AutowarePoseWithUuidStampedDisplay &&) =
    delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateVisualization();

private:
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(
    const autoware_planning_msgs::PoseWithUuidStamped::ConstPtr & meg_ptr) override;

  std::unique_ptr<rviz::Axes> axes_;
  Ogre::SceneNode * uuid_node_{nullptr};
  rviz::MovableText * uuid_{nullptr};

  rviz::FloatProperty * length_property_;
  rviz::FloatProperty * radius_property_;
  rviz::TfFrameProperty * frame_property_;

  rviz::BoolProperty * uuid_text_view_property_;
  rviz::FloatProperty * uuid_text_scale_property_;

  autoware_planning_msgs::PoseWithUuidStamped::ConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__POSE_WITH_UUID_STAMPED__DISPLAY_HPP_
