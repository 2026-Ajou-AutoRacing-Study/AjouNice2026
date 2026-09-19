// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_HPP_

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/rrtstar_core.hpp"

#include <vector>

namespace autoware::freespace_planning_algorithms
{
struct RRTStarParam
{
  // algorithm configs
  bool enable_update;  // update solution even after feasible solution found with given time budget
  bool use_informed_sampling;  // use informed sampling (informed rrtstar)
  double max_planning_time;  // if enable_update is true, update is done before time elapsed [msec]
  double neighbor_radius;    // neighbor radius [m]
  double margin;             // [m]
};

class RRTStar : public AbstractPlanningAlgorithm
{
public:
  RRTStar(
    const PlannerCommonParam & planner_common_param, const VehicleShape & original_vehicle_shape,
    const RRTStarParam & rrtstar_param, const ros1_compat::ClockNow & clock);

  RRTStar(
    const PlannerCommonParam & planner_common_param, const VehicleShape & original_vehicle_shape,
    const ros::NodeHandle & node)
  : RRTStar(
      planner_common_param, original_vehicle_shape,
      RRTStarParam{
        ros1_compat::require_param<bool>(node, "rrtstar/enable_update"),
        ros1_compat::require_param<bool>(node, "rrtstar/use_informed_sampling"),
        ros1_compat::require_param<double>(node, "rrtstar/max_planning_time"),
        ros1_compat::require_param<double>(node, "rrtstar/neighbor_radius"),
        ros1_compat::require_param<double>(node, "rrtstar/margin")},
      ros1_compat::ros_time_now())
  {
  }

  bool makePlan(
    const geometry_msgs::Pose & start_pose,
    const geometry_msgs::Pose & goal_pose) override;
  bool makePlan(
    const geometry_msgs::Pose & start_pose,
    const std::vector<geometry_msgs::Pose> & goal_candidates) override;
  bool hasObstacleOnTrajectory(const geometry_msgs::PoseArray & trajectory) const override;

private:
  void setRRTPath(const std::vector<rrtstar_core::Pose> & waypoints);

  // algorithm specific param
  const RRTStarParam rrtstar_param_;
};

}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_HPP_
