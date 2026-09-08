#include "autoware/planning_validator/invalid_trajectory_handling_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::planning_validator
{
namespace
{
double distance2d(
  const autoware_planning_msgs::TrajectoryPoint & first,
  const autoware_planning_msgs::TrajectoryPoint & second)
{
  return std::hypot(
    first.pose.position.x - second.pose.position.x,
    first.pose.position.y - second.pose.position.y);
}

double stopping_distance(
  const double current_velocity, const double current_acceleration,
  const double deceleration, const double jerk_limit)
{
  if (deceleration >= 0.0 || jerk_limit <= 0.0) return std::numeric_limits<double>::infinity();
  const double ramp_time = std::max((current_acceleration - deceleration) / jerk_limit, 0.0);
  const double velocity_after_ramp =
    current_velocity + current_acceleration * ramp_time - 0.5 * jerk_limit * ramp_time * ramp_time;
  const double distance_during_ramp =
    current_velocity * ramp_time + 0.5 * current_acceleration * ramp_time * ramp_time -
    jerk_limit * ramp_time * ramp_time * ramp_time / 6.0;
  return distance_during_ramp + std::abs(velocity_after_ramp * velocity_after_ramp / (2.0 * deceleration));
}
}  // namespace

std::optional<autoware_planning_msgs::Trajectory> select_invalid_trajectory(
  const autoware_planning_msgs::Trajectory & current,
  const autoware_planning_msgs::Trajectory * last_valid,
  const InvalidTrajectoryHandlingType handling)
{
  if (handling == InvalidTrajectoryHandlingType::PublishAsIs) return current;
  if (!last_valid) return std::nullopt;
  return *last_valid;
}

autoware_planning_msgs::Trajectory make_soft_stop_trajectory(
  const autoware_planning_msgs::Trajectory & last_valid, const size_t nearest_index,
  const double current_velocity, const double current_acceleration,
  const InvalidTrajectoryHandlingParameters & parameters)
{
  auto result = last_valid;
  if (result.points.empty()) return result;
  const double required_distance = stopping_distance(
    current_velocity, current_acceleration, parameters.soft_stop_deceleration,
    parameters.soft_stop_jerk_lim);
  double accumulated_distance = 0.0;
  const size_t begin = std::min(nearest_index + 1U, result.points.size());
  for (size_t index = begin; index < result.points.size(); ++index) {
    accumulated_distance += distance2d(result.points.at(index - 1U), result.points.at(index));
    if (accumulated_distance >= required_distance ||
      result.points.at(index - 1U).longitudinal_velocity_mps < 0.5F) {
      result.points.at(index).longitudinal_velocity_mps = 0.0F;
      continue;
    }
    const double scaled_velocity = current_velocity *
      (required_distance - accumulated_distance) / required_distance;
    result.points.at(index).longitudinal_velocity_mps = static_cast<float>(std::min(
      scaled_velocity, static_cast<double>(result.points.at(index).longitudinal_velocity_mps)));
  }
  result.points.back().longitudinal_velocity_mps = 0.0F;
  return result;
}
}  // namespace autoware::planning_validator
