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

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <ros/ros.h>

#include <autoware_planning_msgs/PoseWithUuidStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace autoware::mission_planner_universe
{

class ArrivalChecker
{
public:
  using PoseWithUuidStamped = autoware_planning_msgs::PoseWithUuidStamped;
  using PoseStamped = geometry_msgs::PoseStamped;
  explicit ArrivalChecker(ros::NodeHandle * node);
  void set_goal();
  void set_goal(const PoseWithUuidStamped & goal);
  bool is_arrived(const PoseStamped & pose) const;

private:
  double angle_;
  double duration_;
  double arrival_check_lateral_distance_;
  double arrival_check_longitudinal_undershoot_distance_;
  double arrival_check_longitudinal_overshoot_distance_;
  std::optional<PoseWithUuidStamped> goal_with_uuid_;
  autoware::motion_utils::VehicleStopChecker vehicle_stop_checker_;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
