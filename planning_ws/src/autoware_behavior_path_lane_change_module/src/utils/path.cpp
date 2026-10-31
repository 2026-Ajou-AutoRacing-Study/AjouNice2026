#include "autoware/behavior_path_lane_change_module/utils/candidate_sampling.hpp"
// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/reentry.hpp"
#include <tf2/utils.h>

#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/speed_profile.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_frenet_planner/frenet_planner.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <range/v3/action/insert.hpp>
#include <range/v3/action/remove_if.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view.hpp>

#include <fmt/format.h>
#include <std_msgs/String.h>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <limits>
#include <vector>

namespace
{
using autoware::behavior_path_planner::LaneChangeInfo;
using autoware::behavior_path_planner::PathPointWithLaneId;
using autoware::behavior_path_planner::PathShifter;
using autoware::behavior_path_planner::PathWithLaneId;
using autoware::behavior_path_planner::ShiftedPath;
using autoware::behavior_path_planner::lane_change::CommonDataPtr;
using autoware::behavior_path_planner::lane_change::LCParamPtr;

using autoware::behavior_path_planner::LaneChangePhaseMetrics;
using autoware::behavior_path_planner::ShiftLine;
using autoware::behavior_path_planner::lane_change::TrajectoryGroup;
using autoware::frenet_planner::FrenetState;
using autoware::frenet_planner::SamplingParameters;
using autoware::route_handler::Direction;
using autoware::sampler_common::FrenetPoint;
using autoware::sampler_common::transform::Spline2D;
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using geometry_msgs::Pose;

template <typename T>
T sign(const Direction direction)
{
  return static_cast<T>(direction == Direction::LEFT ? 1.0 : -1.0);
}

double calc_resample_interval(
  const double lane_changing_length, const double lane_changing_velocity)
{
  constexpr auto min_resampling_points{30.0};
  constexpr auto resampling_dt{0.2};
  return std::max(
    lane_changing_length / min_resampling_points, lane_changing_velocity * resampling_dt);
}

PathWithLaneId get_reference_path_from_target_lane(
  const CommonDataPtr & common_data_ptr, const Pose & lane_changing_start_pose,
  const double lane_changing_length, const double resample_interval)
{
  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto target_lane_length = common_data_ptr->transient_data.target_lane_length;
  const auto is_goal_in_route = common_data_ptr->lanes_ptr->target_lane_in_goal_section;
  const auto next_lc_buffer = common_data_ptr->transient_data.next_dist_buffer.min;
  const auto forward_path_length = common_data_ptr->bpp_param_ptr->forward_path_length;

  const auto lane_change_start_arc_position =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(
      target_lanes, lane_changing_start_pose);

  const double s_start = lane_change_start_arc_position.length;
  const double s_end = std::invoke([&]() {
    const auto dist_from_lc_start = s_start + lane_changing_length + forward_path_length;
    if (is_goal_in_route) {
      const double s_goal = autoware::experimental::lanelet2_utils::get_arc_coordinates(
                              target_lanes, route_handler.getGoalPose())
                              .length -
                            next_lc_buffer;
      return std::min(dist_from_lc_start, s_goal);
    }
    return std::min(dist_from_lc_start, target_lane_length - next_lc_buffer);
  });

  constexpr double epsilon = 1e-4;
  if (s_end - s_start + epsilon < lane_changing_length) {
    return PathWithLaneId();
  }

  const auto lane_changing_reference_path =
    route_handler.getCenterLinePath(target_lanes, s_start, s_end);

  return autoware::behavior_path_planner::utils::resamplePathWithSpline(
    lane_changing_reference_path, resample_interval, true, {0.0, lane_changing_length});
}

ShiftLine get_lane_changing_shift_line(
  const Pose & lane_changing_start_pose, const Pose & lane_changing_end_pose,
  const PathWithLaneId & reference_path, const double shift_length)
{
  ShiftLine shift_line;
  shift_line.end_shift_length = shift_length;
  shift_line.start = lane_changing_start_pose;
  shift_line.end = lane_changing_end_pose;
  shift_line.start_idx = autoware::motion_utils::findNearestIndex(
    reference_path.points, lane_changing_start_pose.position);
  shift_line.end_idx = autoware::motion_utils::findNearestIndex(
    reference_path.points, lane_changing_end_pose.position);

  return shift_line;
}

ShiftedPath get_shifted_path(
  const PathWithLaneId & target_lane_reference_path, const LaneChangeInfo & lane_change_info)
{
  const auto longitudinal_acceleration = lane_change_info.longitudinal_acceleration;

  PathShifter path_shifter;
  path_shifter.setPath(target_lane_reference_path);
  path_shifter.addShiftLine(lane_change_info.shift_line);
  path_shifter.setLongitudinalAcceleration(longitudinal_acceleration.lane_changing);
  const auto initial_lane_changing_velocity = lane_change_info.velocity.lane_changing;
  path_shifter.setVelocity(initial_lane_changing_velocity);
  path_shifter.setLateralAccelerationLimit(std::abs(lane_change_info.lateral_acceleration));

  constexpr auto offset_back = false;
  ShiftedPath shifted_path;
  [[maybe_unused]] const auto success = path_shifter.generate(&shifted_path, offset_back);
  return shifted_path;
}

std::optional<double> exceed_yaw_threshold(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & lane_changing_segment,
  const double yaw_th_rad)
{
  const auto & prepare = prepare_segment.points;
  const auto & lane_changing = lane_changing_segment.points;

  if (prepare.size() <= 1 || lane_changing.size() <= 1) {
    return std::nullopt;
  }

  const auto & p1 = std::prev(prepare.end() - 1)->point.pose;
  const auto & p2 = std::next(lane_changing.begin())->point.pose;
  const auto yaw_diff_rad = std::abs(
    autoware_utils::normalize_radian(tf2::getYaw(p1.orientation) - tf2::getYaw(p2.orientation)));
  if (yaw_diff_rad > yaw_th_rad) {
    return yaw_diff_rad;
  }
  return std::nullopt;
}

Spline2D init_reference_spline(const std::vector<PathPointWithLaneId> & target_lanes_ref_path)
{
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(target_lanes_ref_path.size());
  ys.reserve(target_lanes_ref_path.size());
  for (const auto & p : target_lanes_ref_path) {
    xs.push_back(p.point.pose.position.x);
    ys.push_back(p.point.pose.position.y);
  }

  return {xs, ys};
}

FrenetState init_frenet_state(
  const FrenetPoint & start_position, const LaneChangePhaseMetrics & prepare_metrics)
{
  FrenetState initial_state;
  initial_state.position = start_position;
  initial_state.longitudinal_velocity = prepare_metrics.velocity;
  initial_state.lateral_velocity =
    0.0;  // TODO(Maxime): this can be sampled if we want but it would impact the LC duration
  initial_state.longitudinal_acceleration = prepare_metrics.sampled_lon_accel;
  initial_state.lateral_acceleration = prepare_metrics.lat_accel;
  return initial_state;
}

std::optional<SamplingParameters> init_sampling_parameters(
  const CommonDataPtr & common_data_ptr, const LaneChangePhaseMetrics & prepare_metrics,
  const FrenetState & initial_state, const Spline2D & ref_spline, const double max_lateral_acc,
  const double max_path_velocity)
{
  const auto & trajectory = common_data_ptr->lc_param_ptr->trajectory;
  const auto min_lc_vel = trajectory.min_lane_changing_velocity;
  const auto initial_velocity = prepare_metrics.velocity;
  auto lon_accel = prepare_metrics.sampled_lon_accel;

  const auto use_remaining_distance =
    common_data_ptr->lc_param_ptr->frenet.use_entire_remaining_distance;

  const auto [lc_length, duration, final_velocity] = std::invoke([&]() {
    auto duration = autoware::motion_utils::calc_shift_time_from_jerk(
      std::abs(initial_state.position.d), trajectory.lateral_jerk, max_lateral_acc);
    if (trajectory.min_prepare_duration <= 1e-6) {
      lon_accel = autoware::behavior_path_planner::utils::lane_change::calculation::calc_lane_changing_acceleration(
        common_data_ptr, initial_velocity, max_path_velocity, duration, lon_accel);
    }
    auto final_velocity = std::max(min_lc_vel, initial_velocity + lon_accel * duration);
    auto length = duration * (initial_velocity + final_velocity) * 0.5;
    if (!use_remaining_distance) return std::make_tuple(length, duration, final_velocity);
    const auto dist_lc_start_to_terminal_end =
      common_data_ptr->transient_data.dist_to_terminal_end - prepare_metrics.length -
      common_data_ptr->lc_param_ptr->lane_change_finish_judge_buffer;
    length = std::max(length, dist_lc_start_to_terminal_end);
    final_velocity = std::max(
      min_lc_vel, std::sqrt((initial_velocity * initial_velocity) + (2.0 * lon_accel * length)));
    duration = 2.0 * length / (initial_velocity + final_velocity);
    return std::make_tuple(length, duration, final_velocity);
  });

  // A zero lateral shift must not produce a zero-duration polynomial or 0/0.
  if (!trajectory.allow_decelerating_lane_change && lon_accel < -1e-6) {
    return std::nullopt;
  }
  if (!std::isfinite(duration) || duration <= 1e-6 || !std::isfinite(lc_length) ||
      !std::isfinite(lon_accel) || !std::isfinite(final_velocity)) {
    return std::nullopt;
  }
  if (trajectory.min_prepare_duration <= 1e-6 &&
      (initial_state.position.s + lc_length > ref_spline.lastS() + 1e-6 ||
       final_velocity > std::min(max_path_velocity, common_data_ptr->bpp_param_ptr->max_vel) + 1e-6 ||
       final_velocity < min_lc_vel - 1e-6 ||
       std::abs(final_velocity - initial_velocity - lon_accel * duration) > 1e-6)) {
    return std::nullopt;
  }
  const auto target_s = std::min(ref_spline.lastS(), initial_state.position.s + lc_length);
  // for smooth lateral motion we want a constant lateral acceleration profile
  // this means starting from a 0 lateral velocity and setting a positive target lateral velocity
  const auto max_lat_vel = duration * max_lateral_acc;
  const auto target_lat_vel = std::min(max_lat_vel, initial_state.position.d / duration);

  SamplingParameters sampling_parameters;
  const auto & safety = common_data_ptr->lc_param_ptr->safety;
  sampling_parameters.resolution = safety.collision_check.prediction_time_resolution;
  sampling_parameters.parameters.emplace_back();
  sampling_parameters.parameters.back().target_duration = duration;
  sampling_parameters.parameters.back().target_state.position = {target_s, 0.0};
  sampling_parameters.parameters.back().target_state.lateral_velocity =
    std::copysign(target_lat_vel, sign<double>(common_data_ptr->direction));
  sampling_parameters.parameters.back().target_state.lateral_acceleration = 0.0;
  sampling_parameters.parameters.back().target_state.longitudinal_velocity = final_velocity;
  sampling_parameters.parameters.back().target_state.longitudinal_acceleration =
    lon_accel;
  return sampling_parameters;
}

std::vector<double> calc_curvatures(
  const std::vector<PathPointWithLaneId> & points, const Pose & current_pose)
{
  const auto nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, current_pose.position);

  // Ignore all path points behind ego vehicle.
  if (points.size() <= nearest_segment_idx + 2) {
    return {};
  }

  std::vector<double> curvatures;
  curvatures.reserve(points.size() - nearest_segment_idx + 2);
  for (const auto & [p1, p2, p3] : ranges::views::zip(
         points | ranges::views::drop(nearest_segment_idx),
         points | ranges::views::drop(nearest_segment_idx + 1),
         points | ranges::views::drop(nearest_segment_idx + 2))) {
    const auto point1 = autoware_utils::get_point(p1);
    const auto point2 = autoware_utils::get_point(p2);
    const auto point3 = autoware_utils::get_point(p3);

    curvatures.push_back(autoware_utils::calc_curvature(point1, point2, point3));
  }

  return curvatures;
}

double calc_average_curvature(const std::vector<double> & curvatures)
{
  const auto filter_zeros = [](const auto & k) { return k != 0.0; };
  auto filtered_k = curvatures | ranges::views::filter(filter_zeros);
  if (filtered_k.empty()) {
    return 0.0;
  }
  const auto sums_of_curvatures = [](float sum, const double k) { return sum + std::abs(k); };
  const auto sum_of_k = ranges::accumulate(filtered_k, 0.0, sums_of_curvatures);
  const auto count_k = static_cast<double>(ranges::distance(filtered_k));
  return sum_of_k / count_k;
}

LineString2d get_linestring_bound(const lanelet::ConstLanelets & lanes, const Direction direction)
{
  LineString2d line_string;
  const auto & get_bound = [&](const lanelet::ConstLanelet & lane) {
    return (direction == Direction::LEFT) ? lane.leftBound2d() : lane.rightBound2d();
  };

  const auto reserve_size = ranges::accumulate(
    lanes, 0UL,
    [&](auto sum, const lanelet::ConstLanelet & lane) { return sum + get_bound(lane).size(); });
  line_string.reserve(reserve_size);
  for (const auto & lane : lanes) {
    ranges::for_each(get_bound(lane), [&line_string](const auto & point) {
      boost::geometry::append(line_string, Point2d(point.x(), point.y()));
    });
  }
  return line_string;
}

Point2d shift_point(const Point2d & p1, const Point2d & p2, const double offset)
{
  // Calculate the perpendicular vector
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double length = std::sqrt(dx * dx + dy * dy);

  // Normalize and find the perpendicular direction
  double nx = -dy / length;
  double ny = dx / length;

  return {p1.x() + nx * offset, p1.y() + ny * offset};
}

bool check_out_of_bound_paths(
  const CommonDataPtr & common_data_ptr, const std::vector<Pose> & lane_changing_poses,
  const LineString2d & lane_boundary, const Direction direction, LineString2d & path_ls)
{
  const auto distance = (0.5 * common_data_ptr->bpp_param_ptr->vehicle_width + 0.1);
  const auto offset = sign<double>(direction) * distance;  // invert direction
  if (lane_changing_poses.size() <= 2) {
    return true;  // Remove candidates with insufficient poses
  }

  path_ls.reserve(lane_changing_poses.size());

  const auto segments = lane_changing_poses | ranges::views::sliding(2);
  ranges::for_each(segments | ranges::views::drop(1), [&](const auto & segment) {
    const auto & p1 = segment[0].position;
    const auto & p2 = segment[1].position;
    boost::geometry::append(path_ls, shift_point({p2.x, p2.y}, {p1.x, p1.y}, offset));
  });

  return boost::geometry::disjoint(path_ls, lane_boundary);  // Remove if disjoint
}

double calc_limit(const CommonDataPtr & common_data_ptr, const Pose & lc_end_pose)
{
  const auto dist_to_target_end = std::invoke([&]() {
    if (common_data_ptr->lanes_ptr->target_lane_in_goal_section) {
      return autoware::motion_utils::calcSignedArcLength(
        common_data_ptr->target_lanes_path.points, lc_end_pose.position,
        common_data_ptr->route_handler_ptr->getGoalPose().position);
    }
    return autoware::motion_utils::calcSignedArcLength(
      common_data_ptr->target_lanes_path.points, lc_end_pose.position,
      common_data_ptr->target_lanes_path.points.back().point.pose.position);
  });

  // v2 = u2 + 2ad
  // u = sqrt(2ad)
  return std::clamp(
    std::sqrt(
      std::abs(2.0 * common_data_ptr->bpp_param_ptr->min_acc * std::max(dist_to_target_end, 0.0))),
    common_data_ptr->lc_param_ptr->trajectory.min_lane_changing_velocity,
    common_data_ptr->bpp_param_ptr->max_vel);
}

};  // namespace

namespace autoware::behavior_path_planner::utils::lane_change
{
using behavior_path_planner::lane_change::CommonDataPtr;
using behavior_path_planner::lane_change::PathType;

bool get_prepare_segment(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path,
  const double prep_length, PathWithLaneId & prepare_segment)
{
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto backward_path_length = common_data_ptr->bpp_param_ptr->backward_path_length;

  if (current_lanes.empty() || target_lanes.empty()) {
    throw std::logic_error("Empty lanes!");
  }

  prepare_segment = prev_module_path;
  if (prep_length <= 1e-6 && common_data_ptr->lc_param_ptr->trajectory.min_prepare_duration <= 1e-6) {
    // clipPathLength rounds forward to the next path vertex even at length=0.
    // Insert ego's projection first so an immediate maneuver starts here.
    const auto start_idx = autoware::motion_utils::insertTargetPoint(
      common_data_ptr->get_ego_pose(), 0.0, prepare_segment.points, 3.0, 1.0);
    if (!start_idx) return false;
    prepare_segment.points.resize(*start_idx + 1);
    utils::clipPathLength(prepare_segment, *start_idx, 0.0, backward_path_length);
  } else {
    const size_t current_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prepare_segment.points, common_data_ptr->get_ego_pose(), 3.0, 1.0);
    utils::clipPathLength(prepare_segment, current_seg_idx, prep_length, backward_path_length);
  }

  if (prepare_segment.points.empty()) return false;

  const auto & lc_start_pose = prepare_segment.points.back().point.pose;

  // TODO(Quda, Azu): Is it possible to remove these checks if we ensure prepare segment length is
  // larger than distance to target lane start
  if (!is_valid_start_point(common_data_ptr, lc_start_pose)) return false;

  // lane changing start is at the end of prepare segment
  const auto target_length_from_lane_change_start_pose =
    utils::getArcLengthToTargetLanelet(current_lanes, target_lanes.front(), lc_start_pose);

  // Check if the lane changing start point is not on the lanes next to target lanes,
  if (target_length_from_lane_change_start_pose > std::numeric_limits<double>::epsilon()) {
    throw std::logic_error("lane change start is behind target lanelet!");
  }

  const auto curvatures = calc_curvatures(prepare_segment.points, common_data_ptr->get_ego_pose());

  // curvatures may be empty if ego is near the terminal start of the path.
  if (curvatures.empty()) {
    return true;
  }

  const auto average_curvature = calc_average_curvature(curvatures);

  RCLCPP_DEBUG(get_logger(), "average curvature: %.3f", average_curvature);
  return average_curvature <= common_data_ptr->lc_param_ptr->trajectory.th_prepare_curvature;
}

LaneChangePath get_candidate_path(
  const CommonDataPtr & common_data_ptr, const LaneChangePhaseMetrics & prep_metric,
  const LaneChangePhaseMetrics & lc_metric, const PathWithLaneId & prep_segment,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids, const double shift_length)
{
  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;

  const auto resample_interval = calc_resample_interval(lc_metric.length, prep_metric.velocity);

  if (prep_segment.points.empty()) {
    throw std::logic_error("Empty prepare segment!");
  }

  const auto & lc_start_pose = prep_segment.points.back().point.pose;
  const auto target_lane_reference_path = get_reference_path_from_target_lane(
    common_data_ptr, lc_start_pose, lc_metric.length, resample_interval);

  if (target_lane_reference_path.points.empty()) {
    throw std::logic_error("Empty target reference!");
  }

  const auto lc_end_pose = std::invoke([&]() {
    const auto dist_to_lc_start =
      autoware::experimental::lanelet2_utils::get_arc_coordinates(target_lanes, lc_start_pose)
        .length;
    const auto dist_to_lc_end = dist_to_lc_start + lc_metric.length;
    return route_handler.get_pose_from_2d_arc_length(target_lanes, dist_to_lc_end);
  });

  const auto shift_line = get_lane_changing_shift_line(
    lc_start_pose, lc_end_pose, target_lane_reference_path, shift_length);

  LaneChangeInfo lane_change_info{prep_metric, lc_metric, lc_start_pose, lc_end_pose, shift_line};

  if (
    lane_change_info.length.sum() + common_data_ptr->transient_data.next_dist_buffer.min >
    common_data_ptr->transient_data.dist_to_terminal_end) {
    throw std::logic_error("invalid candidate path length!");
  }

  return utils::lane_change::construct_candidate_path(
    common_data_ptr, lane_change_info, prep_segment, target_lane_reference_path, sorted_lane_ids);
}

LaneChangePath construct_candidate_path(
  const CommonDataPtr & common_data_ptr, const LaneChangeInfo & lane_change_info,
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_lane_reference_path,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids)
{
  const auto & shift_line = lane_change_info.shift_line;
  const auto terminal_lane_changing_velocity = lane_change_info.terminal_lane_changing_velocity;

  auto shifted_path = get_shifted_path(target_lane_reference_path, lane_change_info);
  shifted_path.path.header = prepare_segment.header;

  if (shifted_path.path.points.empty()) {
    throw std::logic_error("Failed to generate shifted path.");
  }

  if (shifted_path.path.points.size() < shift_line.end_idx + 1) {
    throw std::logic_error("Path points are removed by PathShifter.");
  }

  const auto lc_end_idx_opt = autoware::motion_utils::findNearestIndex(
    shifted_path.path.points, lane_change_info.lane_changing_end);

  if (!lc_end_idx_opt) {
    throw std::logic_error("Lane change end idx not found on target path.");
  }

  std::vector<int64_t> prev_ids;
  std::vector<int64_t> prev_sorted_ids;
  const bool competition = common_data_ptr->lc_param_ptr->trajectory.min_prepare_duration <= 1e-6;
  const bool spatial_speed = competition &&
    lane_change_info.longitudinal_acceleration.lane_changing > 1e-6;
  double reference_distance = 0.0;
  for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
    auto & point = shifted_path.path.points.at(i);
    if (i > shift_line.start_idx && i <= shift_line.end_idx) {
      reference_distance += autoware_utils::calc_distance2d(
        target_lane_reference_path.points.at(i - 1), target_lane_reference_path.points.at(i));
    }
    if (competition && !spatial_speed && i <= *lc_end_idx_opt) {
      const double v0 = lane_change_info.velocity.lane_changing;
      const double acceleration = lane_change_info.longitudinal_acceleration.lane_changing;
      const double velocity = std::sqrt(std::max(0.0, v0 * v0 + 2.0 * acceleration * reference_distance));
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps, static_cast<float>(velocity));
    }
    if (i < *lc_end_idx_opt) {
      const auto & current_ids = point.lane_ids;
      point.lane_ids =
        replace_with_sorted_ids(current_ids, sorted_lane_ids, prev_ids, prev_sorted_ids);
      if (!competition) {
        point.point.longitudinal_velocity_mps = std::min(
          point.point.longitudinal_velocity_mps, static_cast<float>(terminal_lane_changing_velocity));
      }
      continue;
    }
    const auto nearest_idx =
      autoware::motion_utils::findNearestIndex(target_lane_reference_path.points, point.point.pose);
    point.lane_ids = target_lane_reference_path.points.at(*nearest_idx).lane_ids;
  }

  if (utils::lane_change::is_intersecting_no_lane_change_lines(
        common_data_ptr, lane_change_info.length, shifted_path.path.points)) {
    throw std::logic_error("Intersect no lane change lines.");
  }

  constexpr auto yaw_diff_th = autoware_utils::deg2rad(5.0);
  if (
    const auto yaw_diff_opt =
      exceed_yaw_threshold(prepare_segment, shifted_path.path, yaw_diff_th)) {
    std::stringstream err_msg;
    err_msg << "Excessive yaw difference " << yaw_diff_opt.value() << " which exceeds the "
            << yaw_diff_th << " radian threshold.";
    throw std::logic_error(err_msg.str());
  }

  LaneChangePath candidate_path;
  candidate_path.path = utils::combinePath(prepare_segment, shifted_path.path);
  candidate_path.shifted_path = shifted_path;
  candidate_path.info = lane_change_info;
  candidate_path.type = PathType::ConstantJerk;

  if (spatial_speed) {
    const auto & bpp = *common_data_ptr->bpp_param_ptr;
    const auto & params = common_data_ptr->lc_param_ptr->trajectory;
    const auto & current_ref = common_data_ptr->current_lanes_path;
    const auto & target_ref = common_data_ptr->target_lanes_path;
    if (current_ref.points.size() < 2 || target_ref.points.size() < 2) {
      throw std::logic_error("Missing road speed reference for LC");
    }
    // Use both endpoints of the containing road segment. This deliberately
    // delays an increase by at most one map segment; it never anticipates it.
    const auto road_limit = [](const auto & ref, const auto & pose) {
      const auto idx = autoware::motion_utils::findNearestSegmentIndex(ref.points, pose.position);
      return std::min(ref.points.at(idx).point.longitudinal_velocity_mps,
                      ref.points.at(idx + 1).point.longitudinal_velocity_mps);
    };
    std::vector<double> distances{0.0}, limits;
    const size_t begin = shift_line.start_idx;
    const size_t end = *lc_end_idx_opt;
    for (size_t i = begin; i <= end; ++i) {
      const auto & point = shifted_path.path.points.at(i);
      if (i > begin) distances.push_back(distances.back() + autoware_utils::calc_distance2d(
        shifted_path.path.points.at(i - 1), point));
      limits.push_back(std::min({bpp.max_vel,
        static_cast<double>(point.point.longitudinal_velocity_mps),
        static_cast<double>(road_limit(current_ref, point.point.pose)),
        static_cast<double>(road_limit(target_ref, point.point.pose))}));
    }
    auto velocities = make_speed_profile(
      distances, limits, lane_change_info.velocity.lane_changing,
      std::min({bpp.max_acc, params.max_longitudinal_acc,
                lane_change_info.longitudinal_acceleration.lane_changing}),
      -std::max(bpp.min_acc, params.min_longitudinal_acc), common_data_ptr->acceleration_limits);
    static const double max_jerk = [] {
      double value = 0.0;
      ros::param::get("/planning/scenario_planning/velocity_smoother/normal/max_jerk", value);
      return value;
    }();
    static const double min_jerk = [] {
      double value = 0.0;
      ros::param::get("/planning/scenario_planning/velocity_smoother/normal/min_jerk", value);
      return value;
    }();
    velocities = limit_speed_profile_jerk(distances, velocities,
      common_data_ptr->get_current_accel(), max_jerk, min_jerk,
      std::min({bpp.max_acc, params.max_longitudinal_acc,
                lane_change_info.longitudinal_acceleration.lane_changing}),
      common_data_ptr->acceleration_limits);
    if (*std::min_element(velocities.begin(), velocities.end()) < params.min_lane_changing_velocity) {
      throw std::logic_error("Spatial speed profile falls below minimum LC velocity");
    }
    for (size_t i = begin; i <= end; ++i) {
      shifted_path.path.points.at(i).point.longitudinal_velocity_mps = velocities.at(i - begin);
    }
    candidate_path.path = utils::combinePath(prepare_segment, shifted_path.path);
    candidate_path.shifted_path = shifted_path;
    candidate_path.info.duration.lane_changing = speed_profile_duration(distances, velocities);
    candidate_path.info.terminal_lane_changing_velocity = velocities.back();
    candidate_path.type = PathType::SpatialSpeedProfile;
  }

  return candidate_path;
}

std::vector<lane_change::TrajectoryGroup> generate_frenet_candidates(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path,
  const std::vector<LaneChangePhaseMetrics> & prep_metrics)
{
  std::vector<lane_change::TrajectoryGroup> trajectory_groups;
  autoware_utils::StopWatch<std::chrono::microseconds> sw;

  const auto & transient_data = common_data_ptr->transient_data;
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto direction = common_data_ptr->direction;
  const auto current_lane_boundary = get_linestring_bound(current_lanes, direction);
  // Record rejected geometry only when a diagnostic consumer (e.g. rosbag) subscribes.
  static ros::Publisher geometry_publisher;
  if (ros::isInitialized() && !geometry_publisher) {
    geometry_publisher = ros::NodeHandle("~").advertise<std_msgs::String>(
      "debug/lane_change_geometry", 10);
  }
  const bool audit_to_log = std::getenv("AUTOWARE_LC_GEOMETRY_AUDIT") != nullptr;
  const bool audit_geometry =
    audit_to_log || (geometry_publisher && geometry_publisher.getNumSubscribers() > 0);

  size_t rejected_prepare = 0, empty_reference = 0, rejected_sampling = 0;
  size_t generated = 0, duplicate = 0, rejected_boundary = 0;

  for (const auto & metric : prep_metrics) {
    PathWithLaneId prepare_segment;
    try {
      if (!utils::lane_change::get_prepare_segment(
            common_data_ptr, prev_module_path, metric.length, prepare_segment)) {
        RCLCPP_DEBUG(get_logger(), "Reject: failed to get valid prepare segment!");
        ++rejected_prepare;
        continue;
      }
    } catch (const std::exception & e) {
      ++rejected_prepare;
      RCLCPP_WARN(get_logger(), "%s", e.what());
      break;
    }
    const auto lc_start_pose = prepare_segment.points.back().point.pose;

    const auto dist_to_end_from_lc_start =
      calculation::calc_dist_from_pose_to_terminal_end(
        common_data_ptr, target_lanes, lc_start_pose) -
      common_data_ptr->lc_param_ptr->lane_change_finish_judge_buffer;
    const auto max_lc_len = transient_data.lane_changing_length.max;
    const auto max_lane_changing_length = std::min(dist_to_end_from_lc_start, max_lc_len);

    constexpr auto resample_interval = 0.5;
    const auto target_lane_reference_path = get_reference_path_from_target_lane(
      common_data_ptr, lc_start_pose, max_lane_changing_length, resample_interval);
    if (target_lane_reference_path.points.empty()) {
      ++empty_reference;
      continue;
    }

    std::vector<double> target_ref_path_sums{0.0};
    target_ref_path_sums.reserve(target_lane_reference_path.points.size() - 1);
    double ref_s = 0.0;
    for (auto it = target_lane_reference_path.points.begin();
         std::next(it) != target_lane_reference_path.points.end(); ++it) {
      ref_s += autoware_utils::calc_distance2d(*it, *std::next(it));
      target_ref_path_sums.push_back(ref_s);
    }

    const auto reference_spline = init_reference_spline(target_lane_reference_path.points);

    const auto initial_state = init_frenet_state(
      reference_spline.frenet({lc_start_pose.position.x, lc_start_pose.position.y}), metric);

    RCLCPP_DEBUG(
      get_logger(), "Initial state [s=%2.2f, d=%2.2f, s'=%2.2f, d'=%2.2f, s''=%2.2f, d''=%2.2f]",
      initial_state.position.s, initial_state.position.d, initial_state.longitudinal_velocity,
      initial_state.lateral_velocity, initial_state.longitudinal_acceleration,
      initial_state.lateral_acceleration);

    const auto & trajectory = common_data_ptr->lc_param_ptr->trajectory;
    std::vector<frenet_planner::Trajectory> frenet_trajectories;
    for (const double lateral_acc : lateral_acceleration_samples(
           trajectory, std::max(trajectory.min_lane_changing_velocity, metric.velocity))) {
      const auto sampling_parameters_opt = init_sampling_parameters(
        common_data_ptr, metric, initial_state, reference_spline, lateral_acc,
        target_lane_reference_path.points.front().point.longitudinal_velocity_mps);
      if (!sampling_parameters_opt) continue;
      auto sampled_initial_state = initial_state;
      sampled_initial_state.longitudinal_acceleration =
        sampling_parameters_opt->parameters.front().target_state.longitudinal_acceleration;
      auto samples = frenet_planner::generateTrajectories(
        reference_spline, sampled_initial_state, *sampling_parameters_opt);
      frenet_trajectories.insert(
        frenet_trajectories.end(), std::make_move_iterator(samples.begin()),
        std::make_move_iterator(samples.end()));
    }
    if (frenet_trajectories.empty()) {
      ++rejected_sampling;
      continue;
    }
    generated += frenet_trajectories.size();

    trajectory_groups.reserve(trajectory_groups.size() + frenet_trajectories.size());
    for (const auto & traj : frenet_trajectories) {
      // Check the entire polynomial, not only its initial/terminal sample.
      if (!trajectory.allow_decelerating_lane_change && std::any_of(
            traj.longitudinal_accelerations.begin(), traj.longitudinal_accelerations.end(),
            [](const double a) { return a < -1e-6; })) {
        ++rejected_sampling;
        continue;
      }
      // A Frenet polynomial can overshoot its sampled terminal acceleration.
      // Do not pass an unreachable prediction to the collision checker.
      bool exceeds_propulsion = false;
      if (common_data_ptr->acceleration_limits.enabled()) {
        if (traj.longitudinal_accelerations.size() != traj.longitudinal_velocities.size()) continue;
        for (size_t i = 1; i < traj.longitudinal_accelerations.size(); ++i) {
          if (traj.longitudinal_accelerations[i] >
              common_data_ptr->acceleration_limits.at(traj.longitudinal_velocities[i]) + 1e-3) {
            exceeds_propulsion = true;
            break;
          }
        }
      }
      if (exceeds_propulsion) { ++rejected_sampling; continue; }
      if (!trajectory_groups.empty()) {
        const auto diff = std::abs(
          traj.frenet_points.back().s -
          trajectory_groups.back().lane_changing.frenet_points.back().s);
        if (diff < common_data_ptr->lc_param_ptr->trajectory.th_lane_changing_length_diff) {
          ++duplicate;
          continue;
        }
      }

      LineString2d tested_edge;
      const auto out_of_bound = check_out_of_bound_paths(
        common_data_ptr, traj.poses, current_lane_boundary, direction, tested_edge);

      // Opt-in geometry audit: keep the exact input to the crossing check, including
      // rejected candidates (which never reach the normal candidate-path publisher).
      if (audit_geometry) {
        std::ostringstream audit;
        audit << std::setprecision(12) << "LC_GEOMETRY {\"side\":\""
              << (direction == Direction::LEFT ? "left" : "right")
              << "\",\"stamp\":" << ros::Time::now().toSec()
              << ",\"frame_id\":\"map\",\"rejected\":" << (out_of_bound ? "true" : "false")
              << ",\"reason\":\""
              << (traj.poses.size() <= 2 ? "insufficient_poses" :
                  (tested_edge.size() < 2 ? "insufficient_edge_points" :
                   (out_of_bound ? "disjoint" : "crosses")))
              << "\",\"prepare_length\":" << metric.length
              << ",\"prepare_duration\":" << metric.duration
              << ",\"initial_d\":" << initial_state.position.d
              << ",\"initial_s\":" << initial_state.position.s
              << ",\"reference_yaw\":" << reference_spline.yaw(initial_state.position.s)
              << ",\"initial_v\":" << initial_state.longitudinal_velocity
              << ",\"width\":" << common_data_ptr->bpp_param_ptr->vehicle_width
              << ",\"prepare_end\":[" << lc_start_pose.position.x << ','
              << lc_start_pose.position.y << "],\"current_ids\":[";
        bool first = true;
        for (const auto & lane : current_lanes) {
          if (!first) audit << ',';
          first = false;
          audit << lane.id();
        }
        audit << "],\"target_ids\":[";
        first = true;
        for (const auto & lane : target_lanes) {
          if (!first) audit << ',';
          first = false;
          audit << lane.id();
        }
        audit << "],\"boundary_ids\":[";
        first = true;
        for (const auto & lane : current_lanes) {
          if (!first) audit << ',';
          first = false;
          audit << (direction == Direction::LEFT ? lane.leftBound().id() : lane.rightBound().id());
        }
        audit << "],\"tested_edge\":[";
        first = true;
        for (const auto & point : tested_edge) {
          if (!first) audit << ',';
          first = false;
          audit << '[' << point.x() << ',' << point.y() << ']';
        }
        audit << "],\"boundary\":[";
        first = true;
        for (const auto & point : current_lane_boundary) {
          if (!first) audit << ',';
          first = false;
          audit << '[' << point.x() << ',' << point.y() << ']';
        }
        audit << "],\"reference\":[";
        first = true;
        for (const auto & point : target_lane_reference_path.points) {
          if (!first) audit << ',';
          first = false;
          audit << '[' << point.point.pose.position.x << ',' << point.point.pose.position.y << ']';
        }
        audit << "],\"poses\":[";
        first = true;
        for (const auto & pose : traj.poses) {
          if (!first) audit << ',';
          first = false;
          audit << '[' << pose.position.x << ',' << pose.position.y << ']';
        }
        audit << "]}";
        if (geometry_publisher && geometry_publisher.getNumSubscribers() > 0) {
          std_msgs::String message;
          message.data = audit.str().substr(std::string("LC_GEOMETRY ").size());
          geometry_publisher.publish(message);
        }
        if (audit_to_log) RCLCPP_INFO(get_logger(), "%s", audit.str().c_str());
      }

      if (out_of_bound) {
        ++rejected_boundary;
        continue;
      }

      const auto lane_changing_average_curvature = calc_average_curvature(traj.curvatures);
      auto sampled_initial_state = initial_state;
      sampled_initial_state.longitudinal_acceleration = traj.longitudinal_accelerations.front();
      trajectory_groups.emplace_back(
        prepare_segment, target_lane_reference_path, target_ref_path_sums, metric, traj,
        sampled_initial_state, lane_changing_average_curvature, max_lane_changing_length);
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "LC Frenet side=%s prepare_samples=%zu prepare_rejected=%zu empty_reference=%zu "
    "sampling_rejected=%zu generated=%zu duplicate=%zu boundary_rejected=%zu retained=%zu",
    direction == Direction::LEFT ? "left" : "right", prep_metrics.size(), rejected_prepare,
    empty_reference, rejected_sampling, generated, duplicate, rejected_boundary,
    trajectory_groups.size());

  const auto limit_vel = [&](TrajectoryGroup & group) {
    const auto max_vel = calc_limit(common_data_ptr, group.lane_changing.poses.back());
    for (auto & vel : group.lane_changing.longitudinal_velocities) {
      vel = std::clamp(vel, 0.0, max_vel);
    }
  };

  ranges::for_each(trajectory_groups, limit_vel);

  ranges::sort(trajectory_groups, [&](const auto & p1, const auto & p2) {
    if (common_data_ptr->lc_param_ptr->trajectory.min_prepare_duration <= 1e-6 &&
        std::abs(p1.prepare_metric.duration - p2.prepare_metric.duration) > 1e-6) {
      return p1.prepare_metric.duration < p2.prepare_metric.duration;
    }
    return p1.lc_average_curvature < p2.lc_average_curvature;
  });

  if (trajectory_groups.size() > 1) {
    auto subrange = ranges::subrange(trajectory_groups.begin() + 1, trajectory_groups.end());

    auto remove_start = ranges::remove_if(subrange, [&](const auto & traj) {
      return traj.lc_average_curvature > common_data_ptr->lc_param_ptr->frenet.th_average_curvature;
    });

    trajectory_groups.erase(remove_start, trajectory_groups.end());
  }

  return trajectory_groups;
}

std::optional<LaneChangePath> get_candidate_path(
  const TrajectoryGroup & trajectory_group, const CommonDataPtr & common_data_ptr,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids)
{
  if (trajectory_group.lane_changing.frenet_points.empty()) {
    return std::nullopt;
  }

  ShiftedPath shifted_path;
  std::vector<int64_t> prev_ids;
  std::vector<int64_t> prev_sorted_ids;
  const auto & lane_changing_candidate = trajectory_group.lane_changing;
  const auto & target_lane_ref_path = trajectory_group.target_lane_ref_path;
  const auto & prepare_segment = trajectory_group.prepare;
  shifted_path.path.header = prepare_segment.header;
  const auto & prepare_metric = trajectory_group.prepare_metric;
  const auto & initial_state = trajectory_group.initial_state;
  const auto & target_ref_sums = trajectory_group.target_lane_ref_path_dist;
  auto zipped_candidates = ranges::views::zip(
    lane_changing_candidate.poses, lane_changing_candidate.frenet_points,
    lane_changing_candidate.longitudinal_velocities, lane_changing_candidate.lateral_velocities,
    lane_changing_candidate.curvatures);

  shifted_path.path.points.reserve(zipped_candidates.size());
  shifted_path.shift_length.reserve(zipped_candidates.size());
  for (const auto & [pose, frenet_point, longitudinal_velocity, lateral_velocity, curvature] :
       zipped_candidates) {
    // Find the reference index
    const auto & s = frenet_point.s;
    auto ref_i_itr = std::find_if(
      target_ref_sums.begin(), target_ref_sums.end(),
      [s](const double ref_s) { return ref_s > s; });
    auto ref_i = std::min<size_t>(
      std::distance(target_ref_sums.begin(), ref_i_itr), target_lane_ref_path.points.size() - 1);

    PathPointWithLaneId point;
    point.point.pose = pose;
    point.point.longitudinal_velocity_mps = static_cast<float>(longitudinal_velocity);
    point.point.lateral_velocity_mps = static_cast<float>(lateral_velocity);
    point.point.heading_rate_rps = static_cast<float>(curvature);
    point.point.pose.position.z = target_lane_ref_path.points[ref_i].point.pose.position.z;
    const auto & current_ids = target_lane_ref_path.points[ref_i].lane_ids;
    point.lane_ids =
      replace_with_sorted_ids(current_ids, sorted_lane_ids, prev_ids, prev_sorted_ids);

    // Add to shifted path
    shifted_path.shift_length.push_back(frenet_point.d);
    shifted_path.path.points.push_back(point);
  }

  if (shifted_path.path.points.empty()) {
    return std::nullopt;
  }

  if (common_data_ptr->lc_param_ptr->trajectory.min_prepare_duration > 1e-6) {
    for (auto & point : shifted_path.path.points) {
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps,
        static_cast<float>(shifted_path.path.points.back().point.longitudinal_velocity_mps));
    }
  }

  const auto th_yaw_diff_deg = common_data_ptr->lc_param_ptr->frenet.th_yaw_diff_deg;
  if (
    const auto yaw_diff_opt = exceed_yaw_threshold(
      prepare_segment, shifted_path.path, autoware_utils::deg2rad(th_yaw_diff_deg))) {
    const auto yaw_diff_deg = autoware_utils::rad2deg(yaw_diff_opt.value());
    const auto err_msg = fmt::format(
      "Excessive yaw difference {yaw_diff:2.2f}[deg]. The threshold is {th_yaw_diff:2.2f}[deg].",
      fmt::arg("yaw_diff", yaw_diff_deg), fmt::arg("th_yaw_diff", th_yaw_diff_deg));
    throw std::logic_error(err_msg);
  }

  LaneChangeInfo info;
  info.longitudinal_acceleration = {
    prepare_metric.actual_lon_accel, lane_changing_candidate.longitudinal_accelerations.front()};
  info.velocity = {prepare_metric.velocity, initial_state.longitudinal_velocity};
  info.duration = {
    prepare_metric.duration, lane_changing_candidate.sampling_parameter.target_duration};
  info.length = {prepare_metric.length, lane_changing_candidate.lengths.back()};
  info.lane_changing_start = prepare_segment.points.back().point.pose;
  info.lane_changing_end = lane_changing_candidate.poses.back();

  if (utils::lane_change::is_intersecting_no_lane_change_lines(
        common_data_ptr, info.length, shifted_path.path.points)) {
    throw std::logic_error("Intersect no lane change lines.");
  }

  ShiftLine sl;

  sl.start = lane_changing_candidate.poses.front();
  sl.end = lane_changing_candidate.poses.back();
  sl.start_shift_length = 0.0;
  sl.end_shift_length = initial_state.position.d;
  sl.start_idx = 0UL;
  sl.end_idx = shifted_path.shift_length.size() - 1;

  info.shift_line = sl;
  info.terminal_lane_changing_velocity = lane_changing_candidate.longitudinal_velocities.back();
  info.lateral_acceleration = lane_changing_candidate.lateral_accelerations.front();

  LaneChangePath candidate_path;
  candidate_path.path = utils::combinePath(prepare_segment, shifted_path.path);
  candidate_path.info = info;
  candidate_path.shifted_path = shifted_path;
  candidate_path.frenet_path = trajectory_group;
  candidate_path.type = PathType::FrenetPlanner;

  return candidate_path;
}

void append_target_ref_to_candidate(
  LaneChangePath & frenet_candidate, const double th_curvature_smoothing)
{
  const auto & target_lane_ref_path = frenet_candidate.frenet_path.target_lane_ref_path.points;
  const auto & lc_end_pose = frenet_candidate.info.lane_changing_end;
  const auto lc_end_idx =
    motion_utils::findNearestIndex(target_lane_ref_path, lc_end_pose.position);
  auto & candidate_path = frenet_candidate.path.points;
  const auto & points = target_lane_ref_path;
  if (target_lane_ref_path.size() <= lc_end_idx + 2) {
    for (const auto & pt : points | ranges::views::drop(lc_end_idx + 1)) {
      candidate_path.push_back(pt);
    }
    return;
  }
  const auto add_size = target_lane_ref_path.size() - (lc_end_idx + 1);
  candidate_path.reserve(candidate_path.size() + add_size);
  for (const auto & [p2, p3] : ranges::views::zip(
         points | ranges::views::drop(lc_end_idx + 1),
         points | ranges::views::drop(lc_end_idx + 2))) {
    const auto point1 = autoware_utils::get_point(candidate_path.back().point.pose);
    const auto point2 = autoware_utils::get_point(p2);
    const auto point3 = autoware_utils::get_point(p3);
    const auto curvature = std::abs(autoware_utils::calc_curvature(point1, point2, point3));
    if (curvature < th_curvature_smoothing) {
      candidate_path.push_back(p2);
    }
  }
  candidate_path.push_back(target_lane_ref_path.back());
}

namespace
{
std::optional<LaneChangePath> make_current_state_transition(
  const CommonDataPtr & common, const double duration, const double desired_acceleration,
  const bool returning, std::string * rejection_reason)
{
  const auto reject = [&](const char * reason) -> std::optional<LaneChangePath> {
    if (rejection_reason) *rejection_reason=reason;
    return std::nullopt;
  };
  if (!common || !common->current_acceleration || !common->is_data_available() || !common->is_lanes_available() ||
      !std::isfinite(duration) || duration <= 0.0 || !std::isfinite(desired_acceleration)) {
    return reject("input_unavailable");
  }
  const auto & params=*common->lc_param_ptr;
  const auto & bpp=*common->bpp_param_ptr;
  if (!returning && !params.trajectory.allow_decelerating_lane_change && desired_acceleration < -1e-6) {
    return reject("decelerating_lc_disabled");
  }
  static const auto jerk_limits=[] {
    std::pair<double,double> limits{0.0,0.0};
    ros::param::get("/planning/scenario_planning/velocity_smoother/normal/max_jerk",limits.first);
    ros::param::get("/planning/scenario_planning/velocity_smoother/normal/min_jerk",limits.second);
    return limits;
  }();
  if (jerk_limits.first<=0.0 || jerk_limits.second>=0.0) return reject("jerk_configuration");
  const auto ego=common->get_ego_pose();
  const auto ref = [&]() {
    if (!returning) return get_reference_path_from_target_lane(common,ego,0.0,0.5);
    const auto & lanes = common->lanes_ptr->current;
    const auto start = autoware::experimental::lanelet2_utils::get_arc_coordinates(lanes,ego).length;
    const auto end = std::min(start+bpp.forward_path_length,
      lanelet::geometry::length3d(lanelet::LaneletSequence(lanes)) -
      common->transient_data.current_dist_buffer.min);
    if (end <= start) return PathWithLaneId{};
    return autoware::behavior_path_planner::utils::resamplePathWithSpline(
      common->route_handler_ptr->getCenterLinePath(lanes,start,end),0.5,true,{0.0,0.0});
  }();
  if (ref.points.size()<3 || common->current_lanes_path.points.size()<2) return reject("reference_unavailable");
  const auto spline=init_reference_spline(ref.points);
  const auto initial=spline.frenet({ego.position.x,ego.position.y});
  const double ref_yaw=spline.yaw(initial.s), curvature=spline.curvature(initial.s);
  const double low=std::max(spline.firstS(),initial.s-0.1);
  const double high=std::min(spline.lastS(),initial.s+0.1);
  if (high<=low) return reject("curvature_reference");
  const double curvature_derivative=(spline.curvature(high)-spline.curvature(low))/(high-low);
  const auto twist=common->get_ego_twist();
  const auto accel=common->current_acceleration->accel.accel.linear;
  const auto motion=reentry_motion(tf2::getYaw(ego.orientation)-ref_yaw,
    twist.linear.x,twist.linear.y,accel.x,accel.y,twist.angular.z,
    initial.d,curvature,curvature_derivative);
  if (!motion) return reject("initial_motion_invalid");
  const auto lateral=ReentryQuintic::to_zero(initial.d,motion->d_velocity,motion->d_acceleration,duration);
  const auto slip=ReentryQuintic::to_zero(motion->slip,motion->slip_rate,0.0,duration);
  if (!lateral || !slip) return reject("initial_polynomial_invalid");
  using Polygon = autoware_utils::Polygon2d;
  using MultiPolygon = boost::geometry::model::multi_polygon<Polygon>;
  MultiPolygon corridor;
  for (const auto * lane : {&common->lanes_polygon_ptr->current,
                           &common->lanes_polygon_ptr->target}) {
    if (lane->size() < 3) continue;
    Polygon polygon;
    for (const auto & point : *lane) polygon.outer().emplace_back(point.x(),point.y());
    boost::geometry::correct(polygon);
    if (!boost::geometry::is_valid(polygon)) return reject("corridor_invalid");
    MultiPolygon merged;
    if (corridor.empty()) corridor.push_back(polygon);
    else { boost::geometry::union_(corridor,polygon,merged); corridor=std::move(merged); }
  }
  if (corridor.empty()) return reject("corridor_missing");
  LaneChangePath candidate;
  candidate.type=behavior_path_planner::lane_change::PathType::SpatialSpeedProfile;
  candidate.path.header=ref.header;
  double s=initial.s, sv=motion->s_velocity, sa=motion->s_acceleration;
  double previous_t=0.0;
  double previous_lat_acc=accel.y;
  std::optional<size_t> ready;
  double ready_time=0.0, arc=0.0, ready_arc=0.0, peak_lat_acc=0.0;
  // Numerical sampling only; constraints and thresholds come from the existing LC configuration.
  const double step=std::min(0.1,params.safety.collision_check.prediction_time_resolution);
  if (!std::isfinite(step) || step<=0.0) return reject("sampling_configuration");
  const size_t count=static_cast<size_t>(std::ceil(duration/step));
  for (size_t i=0;i<=count;++i) {
    const double t=std::min(duration,i*step), dt=t-previous_t;
    if (i) {
      const double maximum=common->acceleration_limits.limit(sv,
        std::min(bpp.max_acc,params.trajectory.max_longitudinal_acc));
      const double desired=std::clamp(desired_acceleration,
        std::max(bpp.min_acc,params.trajectory.min_longitudinal_acc),maximum);
      const double next_acc=sa+std::clamp(desired-sa,jerk_limits.second*dt,jerk_limits.first*dt);
      s+=sv*dt+0.5*sa*dt*dt+(next_acc-sa)*dt*dt/6.0;
      sv+=0.5*(sa+next_acc)*dt;
      sa=next_acc;
    }
    if (s<spline.firstS() || s>spline.lastS() || sv<0.0) return reject("longitudinal_reference_or_reverse");
    const double d=lateral->at(t), dv=lateral->at(t,1);
    const double lateral_jerk_limit = returning ? params.cancel.max_lateral_jerk : params.trajectory.lateral_jerk;
    if (std::abs(lateral->at(t,3))>lateral_jerk_limit+1e-3) return reject("lateral_polynomial_jerk");
    const double scale=1.0-spline.curvature(s)*d;
    if (scale<=0.0) return reject("frenet_scale");
    const auto xy=spline.cartesian(autoware::sampler_common::FrenetPoint{s,d});
    const double tangent=spline.yaw(s);
    const double world_speed=std::hypot(sv*scale,dv), beta=slip->at(t);
    const double vx=world_speed*std::cos(beta);
    if (!std::isfinite(vx) || vx<0.0) return reject("velocity_invalid");
    const double yaw=(i==0) ? tf2::getYaw(ego.orientation) :
      tangent+std::atan2(dv,sv*scale)-beta;
    PathPointWithLaneId point;
    const auto ref_idx=autoware::motion_utils::findNearestSegmentIndex(
      ref.points,autoware_utils::create_point(xy.x(),xy.y(),ego.position.z));
    const auto current_idx=autoware::motion_utils::findNearestSegmentIndex(
      common->current_lanes_path.points,autoware_utils::create_point(xy.x(),xy.y(),ego.position.z));
    const double cap=std::min({bpp.max_vel,
      static_cast<double>(ref.points[ref_idx].point.longitudinal_velocity_mps),
      static_cast<double>(ref.points[ref_idx+1].point.longitudinal_velocity_mps),
      static_cast<double>(common->current_lanes_path.points[current_idx].point.longitudinal_velocity_mps)});
    if (vx>cap+1e-3) return reject("road_speed_limit");
    point.point.pose.position=autoware_utils::create_point(
      xy.x(),xy.y(),ref.points[ref_idx].point.pose.position.z);
    point.point.pose.orientation=autoware_utils::create_quaternion_from_yaw(yaw);
    if (!i) point.point.pose=ego;
    if (!boost::geometry::covered_by(get_ego_footprint(point.point.pose,bpp.vehicle_info),corridor))
      return reject("body_outside_corridor");
    point.point.longitudinal_velocity_mps=vx;
    point.point.lateral_velocity_mps=world_speed*std::sin(beta);
    const double k=spline.curvature(s);
    const double klow=std::max(spline.firstS(),s-0.1), khigh=std::min(spline.lastS(),s+0.1);
    const double kp=(spline.curvature(khigh)-spline.curvature(klow))/(khigh-klow);
    const double u=sv*scale, w=dv;
    const double udot=sa*scale-(kp*sv*d+k*dv)*sv, wdot=lateral->at(t,2);
    const double yaw_rate=i && world_speed>1e-6 ?
      k*sv+(u*wdot-w*udot)/(world_speed*world_speed)-slip->at(t,1) : twist.angular.z;
    const double relative=yaw-tangent;
    const double lat_acc=-(udot-k*sv*w)*std::sin(relative)+
      (wdot+k*sv*u)*std::cos(relative);
    const double max_lat=params.trajectory.lat_acc_map.find(vx).second;
    if (std::abs(lat_acc)>max_lat+1e-3 ||
        (i && std::abs(lat_acc-previous_lat_acc)/dt>lateral_jerk_limit+1e-3)) {
      return reject("lateral_acceleration_or_jerk");
    }
    peak_lat_acc=std::max(peak_lat_acc,std::abs(lat_acc));
    point.point.heading_rate_rps=vx>1e-6 ? yaw_rate/vx : 0.0;
    point.lane_ids=ref.points[ref_idx].lane_ids;
    for (const auto id:common->current_lanes_path.points[current_idx].lane_ids) {
      if (std::find(point.lane_ids.begin(),point.lane_ids.end(),id)==point.lane_ids.end())
        point.lane_ids.push_back(id);
    }
    if (i) arc+=autoware_utils::calc_distance2d(candidate.path.points.back(),point);
    candidate.path.points.push_back(point);
    candidate.shifted_path.shift_length.push_back(d);
    const bool braking=sa < -1e-6;
    const bool speed_ready=vx>=params.trajectory.min_lane_changing_velocity-1e-6;
    if (!ready && (returning || (speed_ready && (params.trajectory.allow_decelerating_lane_change || !braking)))) {
      ready=i;ready_time=t;ready_arc=arc;
    }
    if (!returning && (!speed_ready || (!params.trajectory.allow_decelerating_lane_change && braking)) &&
        !utils::isEgoWithinOriginalLane(common->lanes_polygon_ptr->current,point.point.pose,bpp)) {
      return reject("prepare_motion_outside_original_lane");
    }
    previous_t=t;previous_lat_acc=lat_acc;
  }
  if (!ready || *ready+1>=candidate.path.points.size()) return reject("minimum_lc_velocity");
  const double terminal_buffer = returning ? common->transient_data.current_dist_buffer.min : common->transient_data.next_dist_buffer.min;
  if (arc+terminal_buffer>common->transient_data.dist_to_terminal_end+1e-3)
    return reject("terminal_distance");
  const auto end=candidate.path.points.back().point.pose;
  candidate.info.duration={ready_time,duration-ready_time};
  candidate.info.length={ready_arc,arc-ready_arc};
  candidate.info.velocity={candidate.path.points[*ready].point.longitudinal_velocity_mps,
    candidate.path.points[*ready].point.longitudinal_velocity_mps};
  candidate.info.longitudinal_acceleration={motion->s_acceleration,desired_acceleration};
  candidate.info.terminal_lane_changing_velocity=candidate.path.points.back().point.longitudinal_velocity_mps;
  candidate.info.lateral_acceleration=peak_lat_acc;
  candidate.info.lane_changing_start=candidate.path.points[*ready].point.pose;
  candidate.info.lane_changing_end=end;
  candidate.info.shift_line.start=candidate.info.lane_changing_start;
  candidate.info.shift_line.end=end;
  candidate.info.shift_line.start_idx=*ready;
  candidate.info.shift_line.end_idx=candidate.path.points.size()-1;
  candidate.info.shift_line.end_shift_length=initial.d;
  if (utils::lane_change::is_intersecting_no_lane_change_lines(
      common,candidate.info.length,candidate.path.points)) return reject("no_lane_change_line");
  candidate.shifted_path.path=candidate.path;
  // Keep the normal forward continuation; only the measured-state maneuver is new.
  const auto end_index=autoware::motion_utils::findNearestIndex(ref.points,end.position);
  for (size_t i=end_index+1;i<ref.points.size();++i) candidate.path.points.push_back(ref.points[i]);
  return candidate;
}
}  // namespace

std::optional<LaneChangePath> make_abort_reentry_candidate(
  const CommonDataPtr & common, const double duration, const double acceleration, std::string * rejection)
{
  return make_current_state_transition(common,duration,acceleration,false,rejection);
}

std::optional<LaneChangePath> make_abort_return_candidate(
  const CommonDataPtr & common, const double duration, const double acceleration, std::string * rejection)
{
  return make_current_state_transition(common,duration,acceleration,true,rejection);
}

}  // namespace autoware::behavior_path_planner::utils::lane_change
