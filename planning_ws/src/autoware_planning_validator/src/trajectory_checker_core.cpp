#include "autoware/planning_validator/trajectory_checker_core.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
namespace
{
using Trajectory = autoware_planning_msgs::Trajectory;
using Point = autoware_planning_msgs::TrajectoryPoint;

double distance2d(const Point & a, const Point & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}
double normalized_angle(const double value) { return std::atan2(std::sin(value), std::cos(value)); }
double yaw(const geometry_msgs::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}
double azimuth(const Point & a, const Point & b)
{
  return std::atan2(b.pose.position.y - a.pose.position.y, b.pose.position.x - a.pose.position.x);
}
bool finite(const Point & point)
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) && std::isfinite(o.w) &&
         std::isfinite(point.longitudinal_velocity_mps) && std::isfinite(point.heading_rate_rps) &&
         std::isfinite(point.acceleration_mps2);
}
double curvature(const Point & a, const Point & b, const Point & c)
{
  const double ab = distance2d(a, b), bc = distance2d(b, c), ca = distance2d(c, a);
  const double twice_area = std::abs(
    (b.pose.position.x - a.pose.position.x) * (c.pose.position.y - a.pose.position.y) -
    (b.pose.position.y - a.pose.position.y) * (c.pose.position.x - a.pose.position.x));
  const double denominator = ab * bc * ca;
  return denominator > 1.0e-9 ? 2.0 * twice_area / denominator : 0.0;
}
std::vector<double> curvatures(const Trajectory & trajectory)
{
  std::vector<double> result(trajectory.points.size(), 0.0);
  if (result.size() < 3) return result;
  for (size_t i = 1; i + 1 < result.size(); ++i) {
    result.at(i) = curvature(trajectory.points.at(i - 1), trajectory.points.at(i), trajectory.points.at(i + 1));
  }
  result.front() = result.at(1);
  result.back() = result.at(result.size() - 2);
  return result;
}
double absolute_max(const std::vector<double> & values)
{
  double result = 0.0;
  for (const double value : values) result = std::max(result, std::abs(value));
  return result;
}
size_t nearest_point_index(const Trajectory & trajectory, const geometry_msgs::Point & position)
{
  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i).pose.position;
    const double distance = std::hypot(point.x - position.x, point.y - position.y);
    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }
  return nearest;
}
void mark_context_dependent_checks_unavailable(autoware_planning_validator::PlanningValidatorStatus & status)
{
  // route/map, ego kinematics, vehicle-stop and previous-trajectory inputs
  // are not yet rendered in R002. These must never be claimed as valid.
  status.is_valid_velocity_deviation = false;
  status.is_valid_distance_deviation = false;
  status.is_valid_longitudinal_distance_deviation = false;
  status.is_valid_forward_trajectory_length = false;
  status.is_valid_latency = false;
  status.is_valid_yaw_deviation = false;
  status.is_valid_trajectory_shift = false;
  status.is_valid_intersection_collision_check = false;
  status.is_valid_rear_collision_check = false;
}
}  // namespace

TrajectoryCheckerCore::TrajectoryCheckerCore(TrajectoryCheckerCoreParameters parameters)
: parameters_(std::move(parameters))
{
}

autoware_planning_validator::PlanningValidatorStatus TrajectoryCheckerCore::validate(
  const autoware_planning_msgs::Trajectory & trajectory) const
{
  return validate(TrajectoryCheckerInput{trajectory});
}

autoware_planning_validator::PlanningValidatorStatus TrajectoryCheckerCore::validate(
  const TrajectoryCheckerInput & input) const
{
  const auto & trajectory = input.trajectory;
  autoware_planning_validator::PlanningValidatorStatus status;
  status.stamp = trajectory.header.stamp;
  status.trajectory_size = static_cast<int64_t>(trajectory.points.size());
  mark_context_dependent_checks_unavailable(status);
  status.is_valid_size = trajectory.points.size() >= 2;
  status.is_valid_finite_value = std::all_of(
    trajectory.points.begin(), trajectory.points.end(), [](const Point & point) { return finite(point); });
  if (!status.is_valid_size || !status.is_valid_finite_value) {
    status.invalid_count = 1;
    return status;
  }

  std::vector<double> intervals;
  intervals.reserve(trajectory.points.size() - 1);
  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    intervals.push_back(distance2d(trajectory.points.at(i), trajectory.points.at(i + 1)));
  }
  status.max_interval_distance = absolute_max(intervals);
  status.is_valid_interval = status.max_interval_distance <= parameters_.interval_threshold;

  std::vector<double> angles;
  for (size_t i = 0; i + 2 < trajectory.points.size(); ++i) {
    angles.push_back(std::abs(normalized_angle(
      azimuth(trajectory.points.at(i + 1), trajectory.points.at(i + 2)) -
      azimuth(trajectory.points.at(i), trajectory.points.at(i + 1)))));
  }
  status.max_relative_angle = absolute_max(angles);
  status.is_valid_relative_angle = status.max_relative_angle <= parameters_.relative_angle_threshold;

  const auto curvature_values = curvatures(trajectory);
  status.max_curvature = absolute_max(curvature_values);
  status.is_valid_curvature = status.max_curvature <= parameters_.curvature_threshold;
  std::vector<double> lateral_accels, lateral_jerks, steering_angles;
  lateral_accels.reserve(trajectory.points.size());
  lateral_jerks.reserve(trajectory.points.size());
  steering_angles.reserve(trajectory.points.size());
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);
    lateral_accels.push_back(point.longitudinal_velocity_mps * point.longitudinal_velocity_mps * curvature_values.at(i));
    lateral_jerks.push_back(2.0 * point.longitudinal_velocity_mps * point.acceleration_mps2 * curvature_values.at(i));
    steering_angles.push_back(std::atan(curvature_values.at(i) * parameters_.wheel_base_m));
  }
  status.max_lateral_acc = absolute_max(lateral_accels);
  status.max_lateral_jerk = absolute_max(lateral_jerks);
  status.is_valid_lateral_acc = status.max_lateral_acc <= parameters_.lateral_accel_threshold;
  status.is_valid_lateral_jerk = status.max_lateral_jerk <= parameters_.lateral_jerk_threshold;
  for (const auto & point : trajectory.points) {
    status.max_longitudinal_acc = std::max(status.max_longitudinal_acc, static_cast<double>(point.acceleration_mps2));
    status.min_longitudinal_acc = std::min(status.min_longitudinal_acc, static_cast<double>(point.acceleration_mps2));
  }
  status.is_valid_longitudinal_max_acc = status.max_longitudinal_acc <= parameters_.max_lon_accel_threshold;
  status.is_valid_longitudinal_min_acc = status.min_longitudinal_acc >= parameters_.min_lon_accel_threshold;
  status.max_steering = absolute_max(steering_angles);
  status.is_valid_steering = status.max_steering <= parameters_.steering_threshold;
  std::vector<double> steering_rates;
  for (size_t i = 0; i + 1 < steering_angles.size(); ++i) {
    const double velocity = 0.5 * (trajectory.points.at(i).longitudinal_velocity_mps + trajectory.points.at(i + 1).longitudinal_velocity_mps);
    const double dt = intervals.at(i) / std::max(velocity, 1.0e-5);
    steering_rates.push_back((steering_angles.at(i + 1) - steering_angles.at(i)) / dt);
  }
  status.max_steering_rate = absolute_max(steering_rates);
  status.is_valid_steering_rate = status.max_steering_rate <= parameters_.steering_rate_threshold;
  if (input.current_kinematics) {
    const auto & ego = input.current_kinematics->pose.pose.position;
    const size_t point_index = nearest_point_index(trajectory, ego);
    const size_t segment_index =
      autoware::motion_utils::findNearestSegmentIndex(trajectory.points, ego);
    status.velocity_deviation = std::abs(
      trajectory.points.at(point_index).longitudinal_velocity_mps -
      input.current_kinematics->twist.twist.linear.x);
    status.is_valid_velocity_deviation =
      status.velocity_deviation <= parameters_.velocity_deviation_threshold;
    const auto & first = trajectory.points.at(segment_index).pose.position;
    const auto & second = trajectory.points.at(segment_index + 1).pose.position;
    const double dx = second.x - first.x, dy = second.y - first.y;
    const double length = std::hypot(dx, dy);
    status.distance_deviation = length > 1.0e-9 ?
      ((ego.x - first.x) * dy - (ego.y - first.y) * dx) / length : 0.0;
    status.is_valid_distance_deviation =
      std::abs(status.distance_deviation) <= parameters_.distance_deviation_threshold;
    status.is_valid_longitudinal_distance_deviation = true;
    if (point_index == 0 || point_index == trajectory.points.size() - 1) {
      const size_t edge_segment = point_index == 0 ? 0 : trajectory.points.size() - 2;
      const auto & edge_first = trajectory.points.at(edge_segment).pose.position;
      const auto & edge_second = trajectory.points.at(edge_segment + 1).pose.position;
      const double edge_dx = edge_second.x - edge_first.x, edge_dy = edge_second.y - edge_first.y;
      const double edge_length = std::hypot(edge_dx, edge_dy);
      const double projected = edge_length > 1.0e-9 ?
        ((ego.x - edge_first.x) * edge_dx + (ego.y - edge_first.y) * edge_dy) / edge_length : 0.0;
      status.longitudinal_distance_deviation =
        point_index == 0 ? projected : projected - edge_length;
      status.is_valid_longitudinal_distance_deviation =
        std::abs(status.longitudinal_distance_deviation) <
        parameters_.longitudinal_distance_deviation_threshold;
    }
    const auto & ego_pose = input.current_kinematics->pose.pose;
    const auto interpolated_current =
      autoware::motion_utils::calcInterpolatedPoint(trajectory, ego_pose);
    const double current_yaw = yaw(interpolated_current.pose.orientation);
    status.yaw_deviation = std::abs(normalized_angle(
      current_yaw - yaw(ego_pose.orientation)));
    bool validate_yaw = true;
    if (input.last_valid_trajectory && input.last_valid_trajectory->points.size() >= 2) {
      const auto interpolated_previous = autoware::motion_utils::calcInterpolatedPoint(
        *input.last_valid_trajectory, interpolated_current.pose);
      const double previous_yaw = yaw(interpolated_previous.pose.orientation);
      validate_yaw = std::abs(normalized_angle(current_yaw - previous_yaw)) >
        parameters_.trajectory_yaw_shift_threshold;
    }
    status.is_valid_yaw_deviation = !validate_yaw ||
      status.yaw_deviation <= parameters_.yaw_deviation_threshold;
    if (input.vehicle_stopped) {
      status.is_valid_forward_trajectory_length = true;
    } else {
      status.forward_trajectory_length_measured = autoware::motion_utils::calcSignedArcLength(
        trajectory.points, ego, trajectory.points.size() - 1);
      const double speed = std::abs(input.current_kinematics->twist.twist.linear.x);
      status.forward_trajectory_length_required = speed * speed / (2.0 * 3.0) - 2.0;
      status.is_valid_forward_trajectory_length =
        status.forward_trajectory_length_measured >= status.forward_trajectory_length_required;
    }
    status.is_valid_trajectory_shift = true;
    status.lateral_shift = 0.0;
    status.longitudinal_shift = 0.0;
    if (input.last_valid_trajectory && input.last_valid_trajectory->points.size() >= 2 &&
      !input.vehicle_stopped) {
      const auto & previous = *input.last_valid_trajectory;
      const size_t current_segment =
        autoware::motion_utils::findNearestSegmentIndex(trajectory.points, ego);
      const size_t previous_segment =
        autoware::motion_utils::findNearestSegmentIndex(previous.points, ego);
      const auto & current_pose = trajectory.points.at(current_segment).pose;
      const auto & previous_pose = previous.points.at(previous_segment).pose;
      const double ego_lateral = std::abs(
        autoware_utils_geometry::calc_lateral_deviation(ego_pose, current_pose.position));
      const double lateral_shift = std::abs(
        autoware_utils_geometry::calc_lateral_deviation(previous_pose, current_pose.position));
      status.lateral_shift = lateral_shift > 0.01 ? lateral_shift : 0.0;
      if (ego_lateral > 0.5 && lateral_shift > 0.5) status.is_valid_trajectory_shift = false;
      const bool check_longitudinal = previous_segment != previous.points.size() - 2 &&
        (current_segment == 0 || current_segment == trajectory.points.size() - 2);
      if (check_longitudinal) {
        const double longitudinal_shift = autoware_utils_geometry::calc_longitudinal_deviation(
          previous_pose, current_pose.position);
        status.longitudinal_shift = std::abs(longitudinal_shift) > 0.01 ? longitudinal_shift : 0.0;
        if ((current_segment == 0 && longitudinal_shift > 1.0) ||
          (current_segment == trajectory.points.size() - 2 && longitudinal_shift < -0.1)) {
          status.is_valid_trajectory_shift = false;
        }
      }
    }
  }
  const bool static_valid = status.is_valid_interval && status.is_valid_relative_angle &&
    status.is_valid_curvature && status.is_valid_lateral_acc && status.is_valid_lateral_jerk &&
    status.is_valid_longitudinal_max_acc && status.is_valid_longitudinal_min_acc &&
    status.is_valid_steering && status.is_valid_steering_rate;
  status.invalid_count = static_valid ? 0 : 1;
  return status;
}
}  // namespace autoware::planning_validator
