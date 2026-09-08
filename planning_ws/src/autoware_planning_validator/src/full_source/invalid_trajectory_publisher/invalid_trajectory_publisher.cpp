// Copyright 2021 Tier IV, Inc.
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

#include "invalid_trajectory_publisher.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::planning_validator::full_source
{
InvalidTrajectoryPublisherNode::InvalidTrajectoryPublisherNode(
  const ros::NodeHandle & node_options)
: validator_native::Node(node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  traj_sub_ = subscribe<Trajectory>(
    "input/trajectory", 1,
    &InvalidTrajectoryPublisherNode::onCurrentTrajectory, this);

  traj_pub_ = create_publisher<Trajectory>("output/trajectory", 1);

  timer_ = createTimer(ros::Duration(0.1),
    [this](const ros::TimerEvent &) { onTimer(); });
}

void InvalidTrajectoryPublisherNode::onTimer()
{
  if (!current_trajectory_) {
    RCLCPP_INFO(this->get_logger(), "waiting trajectory");
    return;
  }
  if (current_trajectory_->points.empty()) {
    RCLCPP_INFO(this->get_logger(), "waiting trajectory");
    return;
  }

  constexpr auto ADDED_VALUE = 1.0e3;

  auto output = *current_trajectory_;
  auto & p = output.points.back().pose.position;
  p.x += ADDED_VALUE;
  p.y += ADDED_VALUE;
  p.z += ADDED_VALUE;

  traj_pub_->publish(output);

  RCLCPP_INFO(this->get_logger(), "invalid trajectory is published.");
}

void InvalidTrajectoryPublisherNode::onCurrentTrajectory(const Trajectory::ConstPtr & msg)
{
  current_trajectory_ = msg;
  traj_sub_.shutdown();
}

}  // namespace autoware::planning_validator::full_source
