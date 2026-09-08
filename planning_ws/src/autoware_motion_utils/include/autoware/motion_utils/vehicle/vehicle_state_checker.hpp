// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
#define AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_

#include <ros/ros.h>

#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <deque>

namespace autoware::motion_utils
{

using autoware_planning_msgs::Trajectory;
using geometry_msgs::TwistStamped;
using nav_msgs::Odometry;

class VehicleStopCheckerBase
{
public:
  VehicleStopCheckerBase(ros::NodeHandle * node, double buffer_duration);
  const std::string & getLogger() const { return logger_; }
  void addTwist(const TwistStamped & twist);
  [[nodiscard]] bool isVehicleStopped(const double stop_duration = 0.0) const;

protected:
  std::string logger_;

private:
  double buffer_duration_;
  std::deque<TwistStamped> twist_buffer_;
};

class VehicleStopChecker : public VehicleStopCheckerBase
{
public:
  explicit VehicleStopChecker(ros::NodeHandle * node);

protected:
  ros::Subscriber sub_odom_;
  Odometry::ConstPtr odometry_ptr_;

private:
  static constexpr double velocity_buffer_time_sec = 10.0;
  void onOdom(const Odometry::ConstPtr & msg);
};

class VehicleArrivalChecker : public VehicleStopChecker
{
public:
  explicit VehicleArrivalChecker(ros::NodeHandle * node);

  [[nodiscard]] bool isVehicleStoppedAtStopPoint(const double stop_duration = 0.0) const;

private:
  static constexpr double th_arrived_distance_m = 1.0;

  ros::Subscriber sub_trajectory_;

  Trajectory::ConstPtr trajectory_ptr_;

  void onTrajectory(const Trajectory::ConstPtr & msg);
};
}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
