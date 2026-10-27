// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include "autoware/path_optimizer/utils/gaussian_resampling.hpp"

#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/motion_utils/constants.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::path_optimizer::trajectory_utils
{
namespace
{
constexpr double dense_interval_m = 0.1;
constexpr double pi = 3.14159265358979323846;

std::vector<double> stations(double length, double interval)
{
  if (length / interval > 1000000.0) {
    throw std::invalid_argument("Gaussian resampling exceeds one million samples");
  }
  std::vector<double> values;
  for (size_t i = 0; i * interval < length; ++i) values.push_back(i * interval);
  // Keep both endpoints even on a path shorter than the overlap threshold.
  if (values.size() > 1 && length - values.back() < autoware::motion_utils::overlap_threshold) {
    values.back() = length;
  } else {
    values.push_back(length);
  }
  return values;
}

double anchorBlend(double distance)
{
  const double t = std::clamp((distance - 1.0) / 2.0, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(pi * t);
}
}

std::vector<TrajectoryPoint> resampleTrajectoryPointsGaussian(
  const std::vector<TrajectoryPoint> & points, double output_interval_m,
  double sigma_m, const geometry_msgs::Pose & ego_pose)
{
  if (!std::isfinite(output_interval_m) || output_interval_m <= 0.0 ||
      !std::isfinite(sigma_m) || sigma_m <= 0.0 ||
      !std::isfinite(ego_pose.position.x) || !std::isfinite(ego_pose.position.y)) {
    throw std::invalid_argument("Gaussian output interval and spatial sigma must be finite and positive");
  }
  if (points.size() < 2) return points;

  std::vector<TrajectoryPoint> input;
  for (const auto & point : points) {
    const auto & p = point.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      throw std::invalid_argument("Non-finite position in Gaussian resampling input");
    }
    if (!input.empty() && std::hypot(p.x - input.back().pose.position.x,
                                   p.y - input.back().pose.position.y) < 1e-6) {
      input.back() = point;  // Preserve the last state/stop at duplicate positions.
    } else {
      input.push_back(point);
    }
  }
  if (input.size() < 2) return input;
  const auto trajectory = autoware::motion_utils::convertToTrajectory(input);
  const double length = autoware::motion_utils::calcArcLength(input);
  auto dense_s = stations(length, dense_interval_m);
  // This flag's legacy implementation selects Akima when false.
  const auto dense = autoware::motion_utils::resampleTrajectory(
    trajectory, dense_s, false, true, true);
  if (dense.points.size() != dense_s.size()) {
    throw std::runtime_error("Gaussian support resampling failed");
  }
  auto output_s = stations(length, output_interval_m);
  const auto stop = autoware::motion_utils::calcDistanceToForwardStopPoint(input, 0);
  if (stop && *stop > 0.0 && *stop < length) {
    const auto upper = std::lower_bound(output_s.begin(), output_s.end(), *stop);
    if (upper != output_s.end() && std::abs(*upper - *stop) < 1e-8) {
      // Already present.
    } else if (upper != output_s.begin() && upper != output_s.end() &&
               output_s[std::distance(output_s.begin(), upper) - 1] > 0.0 &&
               *stop - *(upper - 1) < autoware::motion_utils::overlap_threshold) {
      *(upper - 1) = *stop;
    } else if (upper != output_s.end() && *upper < length &&
               *upper - *stop < autoware::motion_utils::overlap_threshold) {
      *upper = *stop;
    } else {
      output_s.insert(upper, *stop);
    }
  }

  size_t ego_index = 0;
  double nearest = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < dense.points.size(); ++i) {
    const auto & p = dense.points[i].pose.position;
    const double distance = std::hypot(p.x - ego_pose.position.x, p.y - ego_pose.position.y);
    if (distance < nearest) { nearest = distance; ego_index = i; }
  }
  const size_t radius = static_cast<size_t>(std::ceil(std::min(
    4.0 * sigma_m / dense_interval_m, static_cast<double>(dense_s.size() - 1))));
  std::vector<double> kernel(radius + 1);
  double weight_sum = 0.0;
  for (size_t j = 0; j <= radius; ++j) {
    const double z = j * dense_interval_m / sigma_m;
    kernel[j] = std::exp(-0.5 * z * z);
    weight_sum += (j == 0 ? 1.0 : 2.0) * kernel[j];
  }
  std::vector<double> x(dense_s.size()), y(dense_s.size());
  for (size_t i = 0; i < dense_s.size(); ++i) {
    double filtered_x = 0.0, filtered_y = 0.0;
    for (std::ptrdiff_t j = -static_cast<std::ptrdiff_t>(radius);
         j <= static_cast<std::ptrdiff_t>(radius); ++j) {
      const size_t index = static_cast<size_t>(std::clamp<std::ptrdiff_t>(
        static_cast<std::ptrdiff_t>(i) + j, 0, dense_s.size() - 1));
      const double weight = kernel[static_cast<size_t>(std::abs(j))];
      filtered_x += weight * dense.points[index].pose.position.x;
      filtered_y += weight * dense.points[index].pose.position.y;
    }
    double blend = anchorBlend(std::min(dense_s[i], length - dense_s[i])) *
      anchorBlend(std::abs(dense_s[i] - dense_s[ego_index]));
    if (stop) blend *= anchorBlend(std::abs(dense_s[i] - *stop));
    const auto & p = dense.points[i].pose.position;
    x[i] = p.x + blend * (filtered_x / weight_sum - p.x);
    y[i] = p.y + blend * (filtered_y / weight_sum - p.y);
  }
  const auto interpolated_x = autoware::interpolation::spline(dense_s, x, output_s);
  const auto interpolated_y = autoware::interpolation::spline(dense_s, y, output_s);
  // Use the original stations for Z/state interpolation and exact stop handling.
  auto output = autoware::motion_utils::resampleTrajectory(
    trajectory, output_s, true, true, true).points;
  if (output.size() != output_s.size()) {
    throw std::runtime_error("Gaussian output state resampling failed");
  }
  for (size_t i = 0; i < output.size(); ++i) {
    // A stop line must keep its physical position, not just its arc station.
    if (stop && std::abs(output_s[i] - *stop) < 1e-8) continue;
    output[i].pose.position.x = interpolated_x[i];
    output[i].pose.position.y = interpolated_y[i];
  }
  autoware::motion_utils::insertOrientation(output, true);
  return output;
}
}
