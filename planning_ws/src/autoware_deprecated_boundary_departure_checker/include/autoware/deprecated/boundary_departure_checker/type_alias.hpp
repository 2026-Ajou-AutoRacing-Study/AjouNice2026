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
#include <autoware/trajectory/trajectory_point.hpp>
#include "autoware/deprecated/boundary_departure_checker/ros1_compat.hpp"
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <autoware_vehicle_msgs/SteeringReport.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <vector>

#ifndef AUTOWARE__DEPRECATED__BOUNDARY_DEPARTURE_CHECKER__TYPE_ALIAS_HPP_
#define AUTOWARE__DEPRECATED__BOUNDARY_DEPARTURE_CHECKER__TYPE_ALIAS_HPP_

namespace autoware::boundary_departure_checker
{
using autoware_internal_planning_msgs::PathWithLaneId;

using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
using autoware_vehicle_msgs::SteeringReport;

using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;  // NOLINT
using autoware_utils_geometry::Polygon2d;
using autoware_utils_geometry::Segment2d;
using autoware_utils_geometry::Segment3d;

using autoware::vehicle_info_utils::VehicleInfo;  // NOLINT

namespace bg = boost::geometry;
namespace bgi = bg::index;                        // NOLINT
using geometry_msgs::Point;
using geometry_msgs::Pose;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__DEPRECATED__BOUNDARY_DEPARTURE_CHECKER__TYPE_ALIAS_HPP_
