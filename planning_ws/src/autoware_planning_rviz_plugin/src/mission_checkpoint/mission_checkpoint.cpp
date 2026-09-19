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

#include "autoware_planning_rviz_plugin/mission_checkpoint/mission_checkpoint.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace rviz_plugins
{
MissionCheckpointTool::MissionCheckpointTool()
{
  shortcut_key_ = 'c';

  pose_topic_property_ = new rviz::StringProperty(
    "Pose Topic", "mission_checkpoint", "The topic on which to publish checkpoint.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  std_dev_x_ = new rviz::FloatProperty(
    "X std deviation", 0.5, "X standard deviation for checkpoint pose [m]", getPropertyContainer());
  std_dev_y_ = new rviz::FloatProperty(
    "Y std deviation", 0.5, "Y standard deviation for checkpoint pose [m]", getPropertyContainer());
  std_dev_theta_ = new rviz::FloatProperty(
    "Theta std deviation", M_PI / 12.0, "Theta standard deviation for checkpoint pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz::FloatProperty(
    "Z position", 0.0, "Z position for checkpoint pose [m]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(0);
}

void MissionCheckpointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Checkpoint Pose");
  updateTopic();
}

void MissionCheckpointTool::updateTopic()
{
  ros::NodeHandle nh;
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
    pose_topic_property_->getStdString(), 1);
}

void MissionCheckpointTool::onPoseSet(double x, double y, double theta)
{
  // pose
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = position_z_->getFloat();

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.orientation = tf2::toMsg(quat);
  ROS_INFO("Setting pose: %.3f %.3f %.3f %.3f [frame=%s]", x, y,
    position_z_->getFloat(), theta, fixed_frame.c_str());
  pose_pub_.publish(pose);
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MissionCheckpointTool, rviz::Tool)
