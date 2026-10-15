// Copyright 2021 Tier IV, Inc.
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

#include "autoware/velocity_smoother/resample.hpp"

#include "autoware/velocity_smoother/ros1_compat.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace autoware::velocity_smoother
{
namespace resampling
{
TrajectoryPoints applyVelocityProfileToGeometry(
  const TrajectoryPoints & geometry, const TrajectoryPoints & profile, size_t ego_index)
{
  ego_index = geometry.empty() ? 0 : std::min(ego_index, geometry.size() - 1);
  const bool emergency_stop = std::all_of(geometry.begin() + ego_index, geometry.end(), [](const auto & p) {
    return std::abs(p.longitudinal_velocity_mps) < 1e-4;
  });
  if (geometry.size() < 2 || profile.size() < 2 || emergency_stop) {
    auto stopped = geometry;
    for (auto & point : stopped) point.longitudinal_velocity_mps = 0.0;
    return stopped;
  }
  std::vector<double> arc(geometry.size(), 0.0);
  for (size_t i = 1; i < geometry.size(); ++i) {
    arc[i] = arc[i - 1] + autoware_utils_geometry::calc_distance2d(geometry[i - 1], geometry[i]);
  }
  // Project the longitudinal solution monotonically onto its original path.
  // Independent nearest-point lookups can jump backwards at close bends.
  std::vector<double> profile_arc;
  profile_arc.reserve(profile.size());
  double previous_s = 0.0;
  for (const auto & point : profile) {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_s = previous_s;
    for (size_t j = 1; j < geometry.size(); ++j) {
      const double length = arc[j] - arc[j - 1];
      if (length < 1e-8 || arc[j] < previous_s) continue;
      const auto & a = geometry[j - 1].pose.position;
      const auto & b = geometry[j].pose.position;
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double lower = std::max(0.0, (previous_s - arc[j - 1]) / length);
      const double ratio = std::clamp(
        ((point.pose.position.x - a.x) * dx + (point.pose.position.y - a.y) * dy) /
          (length * length), lower, 1.0);
      const double distance = std::hypot(
        point.pose.position.x - a.x - ratio * dx,
        point.pose.position.y - a.y - ratio * dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_s = arc[j - 1] + ratio * length;
      }
    }
    previous_s = best_s;
    profile_arc.push_back(best_s);
  }
  const auto first_stop = [](const TrajectoryPoints & points, const std::vector<double> & s,
                             const double start_s, bool moving) {
    for (size_t i = 0; i < points.size(); ++i) {
      if (s[i] < start_s - 1e-8) continue;
      if (std::abs(points[i].longitudinal_velocity_mps) > 1e-4) moving = true;
      else if (moving) return s[i];
    }
    return std::numeric_limits<double>::infinity();
  };
  // Behind-ego velocities are copied from the previous solution for continuity.
  // A past stop followed by resumed motion is history, not a new stop constraint.
  const double ego_s = arc[ego_index];
  const double stop_s = std::min(
    first_stop(geometry, arc, ego_s, true), first_stop(profile, profile_arc, ego_s, false));
  if (std::getenv("AJOUNICE_VS_PROFILE_DIAG")) {
    const double old_stop = std::min(
      first_stop(geometry, arc, 0.0, false), first_stop(profile, profile_arc, 0.0, false));
    if (old_stop + 1e-8 < stop_s) {
      std::cerr << "[VS profile] Ignored history/initial zero: old_s=" << old_stop
                << " ego_s=" << ego_s << " forward_stop_s=" << stop_s << std::endl;
    }
  }
  // Keep all original vertices: coarsening them can cut across an obstacle.
  // Explicitly retain the stop and horizon endpoints between those vertices.
  std::vector<double> queries;
  for (const double s : arc) {
    if (s >= profile_arc.front() && s <= profile_arc.back()) queries.push_back(s);
  }
  queries.push_back(profile_arc.front());
  queries.push_back(profile_arc.back());
  if (stop_s >= profile_arc.front() && stop_s <= profile_arc.back()) queries.push_back(stop_s);
  std::sort(queries.begin(), queries.end());
  queries.erase(std::unique(queries.begin(), queries.end(), [](double a, double b) {
    return std::abs(a - b) < 1e-8;
  }), queries.end());
  TrajectoryPoints result;
  result.reserve(queries.size());
  for (const double s : queries) {
    const auto right = std::upper_bound(arc.begin(), arc.end(), s);
    const size_t j = std::clamp<size_t>(right - arc.begin(), 1, geometry.size() - 1);
    const double ratio = std::clamp(
      (s - arc[j - 1]) / std::max(arc[j] - arc[j - 1], 1e-8), 0.0, 1.0);
    auto point = ratio < 1e-8 ? geometry[j - 1] : geometry[j];
    if (ratio > 1e-8 && ratio < 1.0 - 1e-8) {
      const auto & a = geometry[j - 1].pose;
      const auto & b = geometry[j].pose;
      point.pose.position.x = a.position.x + ratio * (b.position.x - a.position.x);
      point.pose.position.y = a.position.y + ratio * (b.position.y - a.position.y);
      point.pose.position.z = a.position.z + ratio * (b.position.z - a.position.z);
      const auto & qa = a.orientation;
      const auto & qb = b.orientation;
      const auto q = tf2::Quaternion(qa.x, qa.y, qa.z, qa.w).slerp(
        tf2::Quaternion(qb.x, qb.y, qb.z, qb.w), ratio);
      point.pose.orientation.x = q.x(); point.pose.orientation.y = q.y();
      point.pose.orientation.z = q.z(); point.pose.orientation.w = q.w();
    }
    const auto pr = std::upper_bound(profile_arc.begin(), profile_arc.end(), s);
    const size_t k = std::clamp<size_t>(pr - profile_arc.begin(), 1, profile.size() - 1);
    const double t = std::clamp(
      (s - profile_arc[k - 1]) / std::max(profile_arc[k] - profile_arc[k - 1], 1e-8), 0.0, 1.0);
    const auto & a = profile[k - 1];
    const auto & b = profile[k];
    point.longitudinal_velocity_mps = s >= stop_s - 1e-8 ? 0.0 :
      a.longitudinal_velocity_mps + t * (b.longitudinal_velocity_mps - a.longitudinal_velocity_mps);
    point.acceleration_mps2 = a.acceleration_mps2 + t * (b.acceleration_mps2 - a.acceleration_mps2);
    const double time_a = a.time_from_start.sec + 1e-9 * a.time_from_start.nanosec;
    const double time_b = b.time_from_start.sec + 1e-9 * b.time_from_start.nanosec;
    const double time = time_a + t * (time_b - time_a);
    point.time_from_start.sec = static_cast<int32_t>(std::floor(time));
    point.time_from_start.nanosec = static_cast<uint32_t>(
      (time - point.time_from_start.sec) * 1e9);
    result.push_back(point);
  }
  return result;
}

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const double v_current,
  const geometry_msgs::Pose & current_pose, const double nearest_dist_threshold,
  const double nearest_yaw_threshold, const ResampleParam & param, const bool use_zoh_for_v)
{
  // Arc length from the initial point to the closest point
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double negative_front_arclength_value = autoware::motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, input.at(0).pose.position, 0);
  const auto front_arclength_value = std::fabs(negative_front_arclength_value);

  const auto dist_to_closest_stop_point =
    autoware::motion_utils::calcDistanceToForwardStopPoint(input, current_pose);

  // Get the resample size from the closest point
  const double trajectory_length = autoware::motion_utils::calcArcLength(input);
  const double Nt = param.resample_time / std::max(param.dense_resample_dt, 0.001);
  const double ds_nominal =
    std::max(v_current * param.dense_resample_dt, param.dense_min_interval_distance);
  const double Ns = param.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  constexpr double front_ds = 0.1;
  for (double ds = 0.0; ds <= front_arclength_value; ds += front_ds) {
    out_arclength.push_back(ds);
  }
  if (out_arclength.empty()) {
    return input;
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i = 0.0;
  bool is_zero_point_included = false;
  bool is_endpoint_included = false;
  for (size_t i = 1; static_cast<double>(i) <= N; ++i) {
    double ds = ds_nominal;
    if (static_cast<double>(i) > Nt) {
      // if the planning time is not enough to see the desired distance,
      // change the interval distance to see far.
      ds = std::max(param.sparse_min_interval_distance, param.sparse_resample_dt * v_current);
    }

    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > param.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= trajectory_length) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (static_cast<double>(i) > Nt && dist_i >= param.min_trajectory_length) {
      if (
        std::fabs(out_arclength.back() - (param.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = param.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(param.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // Handle Close Stop Point
    if (
      dist_to_closest_stop_point && dist_i > *dist_to_closest_stop_point &&
      !is_zero_point_included) {
      if (std::fabs(dist_i - *dist_to_closest_stop_point) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (*dist_to_closest_stop_point + front_arclength_value)) <
            1e-3) {
          out_arclength.back() = *dist_to_closest_stop_point + front_arclength_value;
        } else {
          out_arclength.push_back(*dist_to_closest_stop_point + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = *dist_to_closest_stop_point;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  if (input.size() < 2 || out_arclength.size() < 2 || trajectory_length < out_arclength.back()) {
    return input;
  }

  const auto output_traj = autoware::motion_utils::resampleTrajectory(
    autoware::motion_utils::convertToTrajectory(input), out_arclength, false, true, use_zoh_for_v);
  auto output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (autoware_utils_geometry::calc_distance2d(output.back(), input.back()) < ep_dist) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }

  return output;
}

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const geometry_msgs::Pose & current_pose,
  const double nearest_dist_threshold, const double nearest_yaw_threshold,
  const ResampleParam & param, const double nominal_ds, const bool use_zoh_for_v)
{
  // input arclength
  const double trajectory_length = autoware::motion_utils::calcArcLength(input);
  const auto dist_to_closest_stop_point =
    autoware::motion_utils::calcDistanceToForwardStopPoint(input, current_pose);

  // distance to stop point
  double stop_arclength_value = param.max_trajectory_length;
  if (dist_to_closest_stop_point) {
    stop_arclength_value = std::min(*dist_to_closest_stop_point, stop_arclength_value);
  }
  if (param.min_trajectory_length < stop_arclength_value) {
    stop_arclength_value = param.min_trajectory_length;
  }

  // Do dense resampling before the stop line(3[m] ahead of the stop line)
  const double start_stop_arclength_value = std::max(stop_arclength_value - 3.0, 0.0);

  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  // Arc length from the initial point to the closest point
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double negative_front_arclength_value = autoware::motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, input.at(0).pose.position,
    static_cast<size_t>(0));
  const auto front_arclength_value = std::fabs(negative_front_arclength_value);
  for (double s = 0.0; s <= front_arclength_value; s += nominal_ds) {
    out_arclength.push_back(s);
  }
  if (out_arclength.empty()) {
    return input;
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i{0.0};
  bool is_zero_point_included{false};
  bool is_endpoint_included{false};
  while (rclcpp::ok()) {
    double ds = nominal_ds;
    if (start_stop_arclength_value <= dist_i && dist_i <= stop_arclength_value) {
      // Dense sampling before the stop point
      ds = 0.01;
    }
    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > param.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= trajectory_length) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (dist_i >= param.min_trajectory_length) {
      if (
        std::fabs(out_arclength.back() - (param.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = param.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(param.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // Handle Close Stop Point
    if (dist_i > stop_arclength_value && !is_zero_point_included) {
      if (std::fabs(dist_i - stop_arclength_value) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (stop_arclength_value + front_arclength_value)) < 1e-3) {
          out_arclength.back() = stop_arclength_value + front_arclength_value;
        } else {
          out_arclength.push_back(stop_arclength_value + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = stop_arclength_value;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  if (input.size() < 2 || out_arclength.size() < 2 || trajectory_length < out_arclength.back()) {
    return input;
  }

  const auto output_traj = autoware::motion_utils::resampleTrajectory(
    autoware::motion_utils::convertToTrajectory(input), out_arclength, false, true, use_zoh_for_v);
  auto output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (autoware_utils_geometry::calc_distance2d(output.back(), input.back()) < ep_dist) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }

  return output;
}

}  // namespace resampling
}  // namespace autoware::velocity_smoother
