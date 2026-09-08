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

#ifndef INVALID_TRAJECTORY_PUBLISHER__INVALID_TRAJECTORY_PUBLISHER_HPP_
#define INVALID_TRAJECTORY_PUBLISHER__INVALID_TRAJECTORY_PUBLISHER_HPP_

#include <autoware/planning_validator/full_source/native_support.hpp>

#include <autoware_planning_msgs/Trajectory.h>

#include <string>

namespace autoware::planning_validator::full_source
{
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;

class InvalidTrajectoryPublisherNode : public validator_native::Node
{
public:
  explicit InvalidTrajectoryPublisherNode(const ros::NodeHandle & node_options);
  void onCurrentTrajectory(const Trajectory::ConstPtr & msg);
  void onTimer();

private:
  // ROS
  ros::Subscriber traj_sub_;
  validator_native::Publisher<Trajectory> traj_pub_;
  ros::Timer timer_;

  Trajectory::ConstPtr current_trajectory_ = nullptr;
};
}  // namespace autoware::planning_validator::full_source

#endif  // INVALID_TRAJECTORY_PUBLISHER__INVALID_TRAJECTORY_PUBLISHER_HPP_
