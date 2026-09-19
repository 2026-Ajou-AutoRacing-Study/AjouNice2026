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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include "obstacle_stop_module.hpp"
#include "type_alias.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <memory>
#include <vector>

namespace test_utils
{

geometry_msgs::Pose create_test_pose(
  double x = 0.0, double y = 0.0, double z = 0.0, double qx = 0.0, double qy = 0.0, double qz = 0.0,
  double qw = 1.0);

std::vector<autoware_planning_msgs::TrajectoryPoint> create_test_trajectory();

std::vector<autoware_planning_msgs::TrajectoryPoint> create_test_decimated_trajectory();

}  // namespace test_utils

#endif  // TEST_UTILS_HPP_
