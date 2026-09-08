// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__FREESPACE_PLANNER__UTILS_HPP_
#define AUTOWARE__FREESPACE_PLANNER__UTILS_HPP_

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include <ros/ros.h>

#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>
#include <vector>

namespace autoware::freespace_planner::utils
{
using autoware::freespace_planning_algorithms::PlannerWaypoint;
using autoware::freespace_planning_algorithms::PlannerWaypoints;
using autoware_internal_planning_msgs::Scenario;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
using geometry_msgs::Pose;
using geometry_msgs::PoseArray;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using nav_msgs::Odometry;

PoseArray trajectory_to_pose_array(const Trajectory & trajectory);

double calc_distance_2d(const Trajectory & trajectory, const Pose & pose);

Pose transform_pose(const Pose & pose, const TransformStamped & transform);

bool is_active(const Scenario::ConstPtr & scenario);

std::vector<size_t> get_reversing_indices(const Trajectory & trajectory);

size_t get_next_target_index(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index);

Trajectory get_partial_trajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t end_index,
  const ros::Time & now);

Trajectory create_trajectory(
  const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity);

Trajectory create_stop_trajectory(
  const PoseStamped & current_pose, const ros::Time & now);

Trajectory create_stop_trajectory(const Trajectory & trajectory);

bool is_stopped(
  const std::deque<Odometry::ConstPtr> & odom_buffer, const double th_stopped_velocity_mps);

bool is_near_target(
  const Pose & target_pose, const Pose & current_pose, const double th_distance_m);
}  // namespace autoware::freespace_planner::utils

#endif  // AUTOWARE__FREESPACE_PLANNER__UTILS_HPP_
