// Copyright 2015-2019 Autoware Foundation
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

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_planning_msgs/PathPoint.h>
namespace {
size_t calcPointIndexFromSegmentIndex(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & point, const size_t seg_idx)
{
  const size_t prev_point_idx = seg_idx;
  const size_t next_point_idx = seg_idx + 1;

  const double prev_dist =
    autoware_utils_geometry::calc_distance2d(point, points.at(prev_point_idx));
  const double next_dist =
    autoware_utils_geometry::calc_distance2d(point, points.at(next_point_idx));

  if (prev_dist < next_dist) {
    return prev_point_idx;
  }
  return next_point_idx;
}

using autoware_planning_msgs::PathPoint;

PathPoint getLerpPathPointWithLaneId(const PathPoint p0, const PathPoint p1, const double ratio)
{
  auto lerp = [](const double a, const double b, const double t) { return a + t * (b - a); };
  PathPoint p;
  p.pose = autoware_utils_geometry::calc_interpolated_pose(p0, p1, ratio);
  const double v = lerp(p0.longitudinal_velocity_mps, p1.longitudinal_velocity_mps, ratio);
  p.longitudinal_velocity_mps = static_cast<float>(v);
  return p;
}

}
namespace autoware::behavior_velocity_planner::planning_utils {
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware::motion_utils::calcSignedArcLength;
using autoware_utils_geometry::calc_distance2d;
bool createDetectionAreaPolygons(
  Polygons2d & da_polys, const PathWithLaneId & path, const geometry_msgs::Pose & target_pose,
  const size_t target_seg_idx, const DetectionRange & da_range, const double obstacle_vel_mps,
  const double min_velocity)
{
  /**
   * @brief relationships for interpolated polygon
   *
   * +(min_length,max_distance)-+ - +---+(max_length,max_distance) = outer_polygons
   * |                                  |
   * +--------------------------+ - +---+(max_length,min_distance) = inner_polygons
   */
  const double min_len = da_range.min_longitudinal_distance;
  const double max_len = da_range.max_longitudinal_distance;
  const double max_dst = da_range.max_lateral_distance;
  const double interval = da_range.interval;
  const double offset_left = (da_range.wheel_tread / 2.0) + da_range.left_overhang;
  const double offset_right = (da_range.wheel_tread / 2.0) + da_range.right_overhang;

  //! max index is the last index of path point
  const auto max_index = static_cast<size_t>(path.points.size() - 1);
  //! avoid bug with same point polygon
  const double eps = 1e-3;
  auto nearest_idx =
    calcPointIndexFromSegmentIndex(path.points, target_pose.position, target_seg_idx);
  if (max_index == nearest_idx) return false;  // case of path point is not enough size
  auto p0 = path.points.at(nearest_idx).point;
  auto first_idx = nearest_idx + 1;

  // use ego point as start point if same point as ego is not in the path
  const auto dist_to_nearest =
    std::fabs(calcSignedArcLength(path.points, target_pose.position, target_seg_idx, nearest_idx));
  if (dist_to_nearest > eps) {
    // interpolate ego point
    const auto & pp = path.points;
    const double ds = calc_distance2d(pp.at(target_seg_idx), pp.at(target_seg_idx + 1));
    const double dist_to_target_seg =
      calcSignedArcLength(path.points, target_seg_idx, target_pose.position, target_seg_idx);
    const double ratio = dist_to_target_seg / ds;
    p0 = getLerpPathPointWithLaneId(
      pp.at(target_seg_idx).point, pp.at(target_seg_idx + 1).point, ratio);

    // new first index should be ahead of p0
    first_idx = target_seg_idx + 1;
  }

  double ttc = 0.0;
  double dist_sum = 0.0;
  double length = 0;
  // initial point of detection area polygon
  LineString2d left_inner_bound = {calculateOffsetPoint2d(p0.pose, min_len, offset_left)};
  LineString2d left_outer_bound = {calculateOffsetPoint2d(p0.pose, min_len, offset_left + eps)};
  LineString2d right_inner_bound = {calculateOffsetPoint2d(p0.pose, min_len, -offset_right)};
  LineString2d right_outer_bound = {calculateOffsetPoint2d(p0.pose, min_len, -offset_right - eps)};
  for (size_t s = first_idx; s <= max_index; s++) {
    const auto p1 = path.points.at(s).point;
    const double ds = calc_distance2d(p0, p1);
    dist_sum += ds;
    length += ds;
    // calculate the distance that obstacles can move until ego reach the trajectory point
    const double v_average = 0.5 * (p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    const double v = std::max(v_average, min_velocity);
    const double dt = ds / v;
    ttc += dt;

    // for offset calculation
    const double max_lateral_distance_right =
      std::min(max_dst, offset_right + ttc * obstacle_vel_mps + eps);
    const double max_lateral_distance_left =
      std::min(max_dst, offset_left + ttc * obstacle_vel_mps + eps);

    // left bound
    if (da_range.use_left) {
      left_inner_bound.emplace_back(calculateOffsetPoint2d(p1.pose, min_len, offset_left));
      left_outer_bound.emplace_back(
        calculateOffsetPoint2d(p1.pose, min_len, max_lateral_distance_left));
    }
    // right bound
    if (da_range.use_right) {
      right_inner_bound.emplace_back(calculateOffsetPoint2d(p1.pose, min_len, -offset_right));
      right_outer_bound.emplace_back(
        calculateOffsetPoint2d(p1.pose, min_len, -max_lateral_distance_right));
    }
    // replace previous point with next point
    p0 = p1;
    // separate detection area polygon with fixed interval or at the end of detection max length
    if (length > interval || max_len < dist_sum || s == max_index) {
      if (left_inner_bound.size() > 1)
        da_polys.emplace_back(lines2polygon(left_inner_bound, left_outer_bound));
      if (right_inner_bound.size() > 1)
        da_polys.emplace_back(lines2polygon(right_outer_bound, right_inner_bound));
      left_inner_bound = {left_inner_bound.back()};
      left_outer_bound = {left_outer_bound.back()};
      right_inner_bound = {right_inner_bound.back()};
      right_outer_bound = {right_outer_bound.back()};
      length = 0;
      if (max_len < dist_sum || s == max_index) return true;
    }
  }
  return true;
}

}
