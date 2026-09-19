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

#include "autoware/path_optimizer/utils/geometry_utils.hpp"
#include "autoware/motion_utils/trajectory/drivable_area_footprint.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_optimizer/mpt_optimizer.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <tf2/utils.h>

#include "autoware_planning_msgs/PathPoint.h"
#include "autoware_planning_msgs/TrajectoryPoint.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <stack>
#include <vector>

namespace autoware_utils_geometry
{

template <>
geometry_msgs::Point get_point(const autoware::path_optimizer::ReferencePoint & p)
{
  return p.pose.position;
}

template <>
geometry_msgs::Pose get_pose(const autoware::path_optimizer::ReferencePoint & p)
{
  return p.pose;
}

template <>
double get_longitudinal_velocity(const autoware::path_optimizer::ReferencePoint & p)
{
  return p.longitudinal_velocity_mps;
}

}  // namespace autoware_utils_geometry

namespace autoware::path_optimizer::geometry_utils
{
bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::Pose & pose, const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const bool use_footprint_polygon_for_outside_drivable_area_check)
{
  return autoware::motion_utils::drivable_area::isOutsideDrivableAreaFromRectangleFootprint(
    pose, left_bound, right_bound, vehicle_info, use_footprint_polygon_for_outside_drivable_area_check);
}
}  // namespace autoware::path_optimizer::geometry_utils
