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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__MISSION_CHECKPOINT__MISSION_CHECKPOINT_HPP_
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__MISSION_CHECKPOINT__MISSION_CHECKPOINT_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#endif

#include <geometry_msgs/PoseStamped.h>

namespace rviz_plugins
{
class MissionCheckpointTool : public rviz::PoseTool
{
  Q_OBJECT

public:
  MissionCheckpointTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:  // NOLINT for Qt
  ros::Publisher pose_pub_;

  rviz::StringProperty * pose_topic_property_;
  rviz::FloatProperty * std_dev_x_;
  rviz::FloatProperty * std_dev_y_;
  rviz::FloatProperty * std_dev_theta_;
  rviz::FloatProperty * position_z_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__MISSION_CHECKPOINT__MISSION_CHECKPOINT_HPP_
