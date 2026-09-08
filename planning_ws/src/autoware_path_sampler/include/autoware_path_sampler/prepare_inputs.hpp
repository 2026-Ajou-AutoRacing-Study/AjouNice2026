// Copyright 2023 Tier IV, Inc.
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

#ifndef AUTOWARE_PATH_SAMPLER__PREPARE_INPUTS_HPP_
#define AUTOWARE_PATH_SAMPLER__PREPARE_INPUTS_HPP_

#include "autoware_frenet_planner/structures.hpp"
#include "autoware_path_sampler/parameters.hpp"
#include "autoware_path_sampler/type_alias.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"

#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <vector>

namespace autoware::path_sampler
{
/// @brief prepare constraints
void prepareConstraints(
  autoware::sampler_common::Constraints & constraints, const PredictedObjects & predicted_objects,
  const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound);
/// @brief prepare sampling parameters to generate Frenet paths
autoware::frenet_planner::SamplingParameters prepareSamplingParameters(
  const autoware::sampler_common::State & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline, const Parameters & params);
/// @brief prepare the 2D spline representation of the given Path message
autoware::sampler_common::transform::Spline2D preparePathSpline(
  const std::vector<TrajectoryPoint> & path_msg, const bool smooth_path);
}  // namespace autoware::path_sampler

#endif  // AUTOWARE_PATH_SAMPLER__PREPARE_INPUTS_HPP_
