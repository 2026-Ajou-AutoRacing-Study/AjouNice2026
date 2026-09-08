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

#ifndef INPUT_HPP_
#define INPUT_HPP_

#endif  // INPUT_HPP_

#include "autoware_internal_planning_msgs/PathPointWithLaneId.h"
#include "autoware_internal_planning_msgs/PathWithLaneId.h"
#include "autoware_planning_msgs/PathPoint.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::PathPointWithLaneId;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_planning_msgs::PathPoint;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
PathWithLaneId generateStraightSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample);

PathWithLaneId generateDiagonalSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample);

Twist generateSampleEgoTwist(float && l_x, float && l_y, float && l_z);

Pose generateEgoSamplePose(float && p_x, float && p_y, float && p_z);
}  // namespace autoware::behavior_path_planner
