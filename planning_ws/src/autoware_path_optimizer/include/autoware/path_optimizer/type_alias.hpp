// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_OPTIMIZER__TYPE_ALIAS_HPP_
#define AUTOWARE__PATH_OPTIMIZER__TYPE_ALIAS_HPP_

#include "autoware_internal_debug_msgs/Float64Stamped.h"
#include "autoware_internal_debug_msgs/StringStamped.h"
#include "autoware_planning_msgs/Path.h"
#include "autoware_planning_msgs/PathPoint.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_planning_msgs/TrajectoryPoint.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/MarkerArray.h"

namespace autoware::path_optimizer
{
// std_msgs
using std_msgs::Header;
// planning
using autoware_planning_msgs::Path;
using autoware_planning_msgs::PathConstPtr;
using autoware_planning_msgs::PathPoint;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
// navigation
using nav_msgs::Odometry;
using nav_msgs::OdometryConstPtr;
// visualization
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
// debug
using autoware_internal_debug_msgs::Float64Stamped;
using autoware_internal_debug_msgs::StringStamped;
}  // namespace autoware::path_optimizer

#endif  // AUTOWARE__PATH_OPTIMIZER__TYPE_ALIAS_HPP_
