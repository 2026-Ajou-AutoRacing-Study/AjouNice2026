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
#include <cmath>
#include <cstdlib>
#include <iostream>
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

bool isOutsideDrivableAreaFromCircleFootprint(
  const geometry_msgs::Pose & pose, const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound,
  const std::vector<double> & circle_radii,
  const std::vector<double> & circle_longitudinal_offsets)
{
  if (left_bound.size() < 2 || right_bound.size() < 2) {
    return false;
  }
  if (circle_radii.empty() || circle_radii.size() != circle_longitudinal_offsets.size()) {
    // A missing or malformed optimization footprint must not silently pass the final check.
    return true;
  }

  using autoware::motion_utils::drivable_area::LineString2d;
  using autoware::motion_utils::drivable_area::Point2d;
  namespace bg = boost::geometry;

  const auto drivable_polygon =
    autoware::motion_utils::drivable_area::createDrivablePolygon(left_bound, right_bound);
  LineString2d left_line;
  LineString2d right_line;
  for (const auto & point : left_bound) {
    left_line.emplace_back(point.x, point.y);
  }
  for (const auto & point : right_bound) {
    right_line.emplace_back(point.x, point.y);
  }
  const LineString2d back_line{left_line.back(), right_line.back()};
  const auto diagnose = [&](const size_t circle_index, const geometry_msgs::Point & center,
                            const double radius, const double left_distance,
                            const double right_distance, const double back_distance,
                            const bool center_inside, const bool before_front) {
    if (std::getenv("AJOUNICE_PO_CIRCLE_DIAG") == nullptr) {
      return;
    }
    static size_t reports = 0;
    if (reports++ >= 8) {
      return;
    }
    std::cerr << "[PO circle-check] circle=" << circle_index << " pose=(" << pose.position.x
              << "," << pose.position.y << ") center=(" << center.x << "," << center.y
              << ") radius=" << radius << " distances=(" << left_distance << ","
              << right_distance << "," << back_distance << ") center_inside=" << center_inside
              << " before_front=" << before_front << std::endl;
  };
  const auto distance_to_line = [](const Point2d & point, const LineString2d & line) {
    double minimum = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < line.size(); ++i) {
      const Eigen::Vector2d segment = line.at(i) - line.at(i - 1);
      const double squared_length = segment.squaredNorm();
      const double fraction = squared_length > 0.0
        ? std::clamp((point - line.at(i - 1)).dot(segment) / squared_length, 0.0, 1.0)
        : 0.0;
      const Eigen::Vector2d nearest = line.at(i - 1) + fraction * segment;
      minimum = std::min(minimum, (point - nearest).norm());
    }
    return minimum;
  };

  for (size_t i = 0; i < circle_radii.size(); ++i) {
    const double radius = circle_radii.at(i);
    if (!std::isfinite(radius) || radius <= 0.0) {
      return true;
    }
    const auto center_pose =
      autoware_utils_geometry::calc_offset_pose(pose, circle_longitudinal_offsets.at(i), 0.0, 0.0);
    const auto & center = center_pose.position;
    const Point2d point{center.x, center.y};
    const bool before_front = autoware::motion_utils::drivable_area::isFrontDrivableArea(
      center, left_bound, right_bound);
    const bool center_inside = bg::within(point, drivable_polygon);
    const double left_distance = distance_to_line(point, left_line);
    const double right_distance = distance_to_line(point, right_line);
    const double back_distance = distance_to_line(point, back_line);
    if (!before_front && !center_inside) {
      diagnose(
        i, center, radius, left_distance, right_distance, back_distance, center_inside,
        before_front);
      return true;
    }
    // A full disk, rather than its center or a sampled polygon, must clear every boundary.
    if (
      left_distance <= radius || right_distance <= radius || back_distance <= radius) {
      diagnose(
        i, center, radius, left_distance, right_distance, back_distance, center_inside,
        before_front);
      return true;
    }
  }
  return false;
}
}  // namespace autoware::path_optimizer::geometry_utils
