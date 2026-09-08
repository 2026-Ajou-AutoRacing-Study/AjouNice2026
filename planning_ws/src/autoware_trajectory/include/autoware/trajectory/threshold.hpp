// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__THRESHOLD_HPP_
#define AUTOWARE__TRAJECTORY__THRESHOLD_HPP_

#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <lanelet2_core/Forward.h>

namespace autoware::experimental::trajectory
{

// TODO(soblin): this should be defined as one of Autoware's interface/limitations
static constexpr double k_epsilon_time = 1e-6;       // second
static constexpr double k_epsilon_distance = 0.005;  // meter
static constexpr double k_epsilon_velocity = 1e-6;   // m/s
static constexpr double k_epsilon_angle = 1e-6;      // rad

static constexpr double k_points_minimum_dist_threshold = k_epsilon_distance;  // deprecated

/**
 * @brief check if two base values are almost-same
 */
bool is_almost_same(const double s1, const double s2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2);

/**
 * @brief check if two quaternions represent almost the same rotation
 */
bool is_almost_same(
  const geometry_msgs::Quaternion & q1, const geometry_msgs::Quaternion & q2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(
  const autoware_planning_msgs::PathPoint & p1,
  const autoware_planning_msgs::PathPoint & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(
  const autoware_planning_msgs::TrajectoryPoint & p1,
  const autoware_planning_msgs::TrajectoryPoint & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(
  const autoware_internal_planning_msgs::PathPointWithLaneId & p1,
  const autoware_internal_planning_msgs::PathPointWithLaneId & p2);

/**
 * @brief check if two points are almost-same
 */
bool is_almost_same(const lanelet::ConstPoint3d & p1, const lanelet::ConstPoint3d & p2);
}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__THRESHOLD_HPP_
