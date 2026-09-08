// Copyright 2024 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <limits>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::virtual_traffic_light
{

using autoware_utils::calc_distance2d;

tier4_v2x_msgs::KeyValue createKeyValue(const std::string & key, const std::string & value)
{
  tier4_v2x_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

autoware_utils::LineString3d toAutowarePoints(const lanelet::ConstLineString3d & line_string)
{
  autoware_utils::LineString3d output;
  for (const auto & p : line_string) {
    output.emplace_back(p.x(), p.y(), p.z());
  }
  return output;
}

std::optional<autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::Optional<lanelet::ConstLineString3d> & line_string)
{
  if (!line_string) {
    return {};
  }
  return toAutowarePoints(*line_string);
}

std::vector<autoware_utils::LineString3d> toAutowarePoints(
  const lanelet::ConstLineStrings3d & line_strings)
{
  std::vector<autoware_utils::LineString3d> output;
  for (const auto & line_string : line_strings) {
    output.emplace_back(toAutowarePoints(line_string));
  }
  return output;
}

autoware_utils::Point3d calcCenter(const autoware_utils::LineString3d & line_string)
{
  const auto p1 = line_string.front();
  const auto p2 = line_string.back();
  const auto p_center = (p1 + p2) / 2;
  return {p_center.x(), p_center.y(), p_center.z()};
}

geometry_msgs::Pose calcHeadPose(
  const geometry_msgs::Pose & base_link_pose, const double base_link_to_front)
{
  return autoware_utils::calc_offset_pose(base_link_pose, base_link_to_front, 0.0, 0.0);
}

geometry_msgs::Point convertToGeomPoint(const autoware_utils::Point3d & p)
{
  geometry_msgs::Point geom_p;
  geom_p.x = p.x();
  geom_p.y = p.y();

  return geom_p;
}

void insertStopVelocityFromStart(autoware_internal_planning_msgs::PathWithLaneId * path)
{
  for (auto & p : path->points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }
}

std::optional<size_t> insertStopVelocityAtCollision(
  const SegmentIndexWithPoint & collision, const double offset,
  autoware_internal_planning_msgs::PathWithLaneId * path)
{
  const auto collision_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
    path->points, collision.index, collision.point);

  const auto offset_segment =
    arc_lane_utils::findOffsetSegment(*path, collision.index, offset + collision_offset);
  if (!offset_segment) {
    return {};
  }

  const auto interpolated_pose = arc_lane_utils::calcTargetPose(*path, *offset_segment);

  if (offset_segment->second < 0) {
    insertStopVelocityFromStart(path);
    return 0;
  }

  const auto same_position = [](const geometry_msgs::Point & lhs, const geometry_msgs::Point & rhs) {
    constexpr double epsilon = 1e-6;
    return std::abs(lhs.x - rhs.x) < epsilon && std::abs(lhs.y - rhs.y) < epsilon &&
           std::abs(lhs.z - rhs.z) < epsilon;
  };

  // insertVelocity in the ROS2 common library replaces an existing point when
  // the target lies on it.  The ROS1 arc helper represents that as either end
  // of a segment, so retain that distinction instead of always duplicating it.
  size_t insert_index = offset_segment->first;
  if (same_position(path->points.at(insert_index).point.pose.position, interpolated_pose.position)) {
    path->points[insert_index].point.pose = interpolated_pose;
  } else if (
    same_position(
      path->points.at(insert_index + 1).point.pose.position, interpolated_pose.position)) {
    ++insert_index;
    path->points[insert_index].point.pose = interpolated_pose;
  } else {
    ++insert_index;
    auto insert_point = path->points.at(insert_index);
    insert_point.point.pose = interpolated_pose;
    path->points.insert(
      path->points.begin() + static_cast<std::ptrdiff_t>(insert_index), insert_point);
  }

  for (size_t i = insert_index; i < path->points.size(); ++i) {
    path->points[i].point.longitudinal_velocity_mps = 0.0;
  }
  return insert_index;
}

}  // namespace autoware::behavior_velocity_planner::virtual_traffic_light
