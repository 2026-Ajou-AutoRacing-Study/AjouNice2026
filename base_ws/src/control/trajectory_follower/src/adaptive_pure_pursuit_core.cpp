#include "trajectory_follower/adaptive_pure_pursuit_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_follower
{
namespace
{
constexpr double kEpsilon = 1e-6;
constexpr double kLaneDirectionDistanceThreshold = 0.05;

enum class LaneDirection
{
  kForward,
  kBackward,
  kInvalid,
};

double clampDouble(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool finitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

double distance2d(const Pose2D & first, const Pose2D & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

double squaredDistance2d(const Pose2D & first, const Pose2D & second)
{
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  return dx * dx + dy * dy;
}

double relativeX(const Pose2D & origin, const Pose2D & point)
{
  return std::cos(origin.yaw) * (point.x - origin.x) +
         std::sin(origin.yaw) * (point.y - origin.y);
}

// Mirrors planning_utils::getLaneDirection() in Autoware Pure Pursuit.  Direction is a
// geometric property of the trajectory poses, not the sign of its target velocity.
LaneDirection getLaneDirection(const std::vector<PathPoint> & path)
{
  if (path.size() < 2U) {
    return LaneDirection::kInvalid;
  }
  const double minimum_distance_squared =
    kLaneDirectionDistanceThreshold * kLaneDirectionDistanceThreshold;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Pose2D & previous = i + 1U == path.size() ? path[i - 1U].pose : path[i].pose;
    const Pose2D & next = i + 1U == path.size() ? path[i].pose : path[i + 1U].pose;
    if (squaredDistance2d(previous, next) > minimum_distance_squared) {
      return relativeX(previous, next) > 0.0 ? LaneDirection::kForward : LaneDirection::kBackward;
    }
  }
  return LaneDirection::kInvalid;
}

Pose2D interpolate(const PathPoint & first, const PathPoint & second, const double ratio)
{
  Pose2D output;
  output.x = first.pose.x + ratio * (second.pose.x - first.pose.x);
  output.y = first.pose.y + ratio * (second.pose.y - first.pose.y);
  output.yaw = normalizeAngle(first.pose.yaw + ratio * normalizeAngle(second.pose.yaw - first.pose.yaw));
  return output;
}

double threePointCurvature(const Pose2D & first, const Pose2D & middle, const Pose2D & last)
{
  const double first_middle = distance2d(first, middle);
  const double middle_last = distance2d(middle, last);
  const double last_first = distance2d(last, first);
  const double denominator = first_middle * middle_last * last_first;
  if (denominator < kEpsilon) {
    return 0.0;
  }
  const double cross = (middle.x - first.x) * (last.y - first.y) -
                       (middle.y - first.y) * (last.x - first.x);
  return 2.0 * cross / denominator;
}

}  // namespace

AdaptivePurePursuitCore::AdaptivePurePursuitCore(const PurePursuitParams & params) : params_(params) {}

void AdaptivePurePursuitCore::setParams(const PurePursuitParams & params)
{
  params_ = params;
}

bool AdaptivePurePursuitCore::parametersValid(std::string * reason) const
{
  const bool common_valid =
    std::isfinite(params_.wheel_base) && params_.wheel_base > 0.0 &&
    std::isfinite(params_.max_steering_angle) && params_.max_steering_angle > 0.0 &&
    std::isfinite(params_.resampling_ds) && params_.resampling_ds > 0.0 &&
    std::isfinite(params_.curvature_calculation_distance) &&
    params_.curvature_calculation_distance > 0.0;
  const bool fixed_valid =
    params_.lookahead_mode == LookaheadMode::kAdaptive ||
    (std::isfinite(params_.fixed_lookahead_distance) &&
     params_.fixed_lookahead_distance > 0.0);
  const bool adaptive_valid =
    params_.lookahead_mode == LookaheadMode::kFixed ||
    (std::isfinite(params_.ld_velocity_ratio) && params_.ld_velocity_ratio >= 0.0 &&
     std::isfinite(params_.ld_lateral_error_ratio) && params_.ld_lateral_error_ratio >= 0.0 &&
     std::isfinite(params_.ld_curvature_ratio) && params_.ld_curvature_ratio >= 0.0 &&
     std::isfinite(params_.long_ld_lateral_error_threshold) &&
     params_.long_ld_lateral_error_threshold >= 0.0 &&
     std::isfinite(params_.min_lookahead_distance) && params_.min_lookahead_distance > 0.0 &&
     std::isfinite(params_.reverse_min_lookahead_distance) &&
     params_.reverse_min_lookahead_distance > 0.0 &&
     std::isfinite(params_.max_lookahead_distance) &&
     params_.max_lookahead_distance >= params_.min_lookahead_distance &&
     params_.max_lookahead_distance >= params_.reverse_min_lookahead_distance);
  const bool valid = common_valid && fixed_valid && adaptive_valid;
  if (!valid && reason) {
    *reason = "invalid pure pursuit parameter";
  }
  return valid;
}

std::vector<PathPoint> AdaptivePurePursuitCore::resamplePath(const std::vector<PathPoint> & path) const
{
  std::vector<PathPoint> filtered;
  filtered.reserve(path.size());
  for (const auto & point : path) {
    if (filtered.empty() || distance2d(filtered.back().pose, point.pose) > kEpsilon) {
      filtered.push_back(point);
    }
  }
  if (filtered.size() < 2U) {
    return filtered;
  }

  std::vector<double> arc_length(filtered.size(), 0.0);
  for (std::size_t i = 1; i < filtered.size(); ++i) {
    arc_length[i] = arc_length[i - 1] + distance2d(filtered[i - 1].pose, filtered[i].pose);
  }
  if (arc_length.back() < kEpsilon) {
    return filtered;
  }

  std::vector<PathPoint> output;
  output.reserve(static_cast<std::size_t>(arc_length.back() / params_.resampling_ds) + 2U);
  std::size_t segment = 0;
  for (double station = 0.0; station < arc_length.back(); station += params_.resampling_ds) {
    while (segment + 1U < arc_length.size() && arc_length[segment + 1U] < station) {
      ++segment;
    }
    if (segment + 1U >= filtered.size()) {
      break;
    }
    const double segment_length = arc_length[segment + 1U] - arc_length[segment];
    if (segment_length < kEpsilon) {
      continue;
    }
    const double ratio = (station - arc_length[segment]) / segment_length;
    PathPoint point;
    point.pose = interpolate(filtered[segment], filtered[segment + 1U], ratio);
    point.target_velocity_mps = filtered[segment].target_velocity_mps + ratio *
      (filtered[segment + 1U].target_velocity_mps - filtered[segment].target_velocity_mps);
    output.push_back(point);
  }
  output.push_back(filtered.back());
  return output;
}

bool AdaptivePurePursuitCore::findNearestIndex(
  const std::vector<PathPoint> & path, const Pose2D & pose, std::size_t * index) const
{
  double minimum_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance_squared = squaredDistance2d(pose, path[i].pose);
    if (distance_squared < minimum_distance_squared) {
      minimum_distance_squared = distance_squared;
      *index = i;
    }
  }
  return std::isfinite(minimum_distance_squared);
}

double AdaptivePurePursuitCore::calculateLateralError(
  const std::vector<PathPoint> & path, const Pose2D & pose) const
{
  // Autoware motion_utils::calcLateralOffset() first chooses the nearest segment, rather
  // than arbitrarily using the segment after the nearest sampled point.
  std::size_t start = 0U;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    const double dx = path[i + 1U].pose.x - path[i].pose.x;
    const double dy = path[i + 1U].pose.y - path[i].pose.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < kEpsilon) {
      continue;
    }
    const double projection = clampDouble(
      ((pose.x - path[i].pose.x) * dx + (pose.y - path[i].pose.y) * dy) / length_squared,
      0.0, 1.0);
    Pose2D closest;
    closest.x = path[i].pose.x + projection * dx;
    closest.y = path[i].pose.y + projection * dy;
    const double distance_squared = squaredDistance2d(closest, pose);
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      start = i;
    }
  }
  const std::size_t end = start + 1U;
  const double dx = path[end].pose.x - path[start].pose.x;
  const double dy = path[end].pose.y - path[start].pose.y;
  const double length = std::hypot(dx, dy);
  if (length < kEpsilon) {
    return 0.0;
  }
  return (dx * (pose.y - path[start].pose.y) - dy * (pose.x - path[start].pose.x)) / length;
}

double AdaptivePurePursuitCore::calculatePathCurvature(
  const std::vector<PathPoint> & path, const std::size_t nearest_index) const
{
  const std::size_t requested_distance = std::max(
    static_cast<std::size_t>(params_.curvature_calculation_distance / params_.resampling_ds),
    static_cast<std::size_t>(1));
  if (path.size() < 3U || nearest_index >= path.size()) {
    return 0.0;
  }
  // Keep a symmetric three-point window where possible and shift the entire
  // window at path boundaries. This avoids reporting a false zero curvature
  // for the first/last curvature-calculation-distance metres.
  const std::size_t index_distance = std::min(requested_distance, (path.size() - 1U) / 2U);
  if (index_distance == 0U) {
    return 0.0;
  }
  const std::size_t maximum_start = path.size() - 1U - 2U * index_distance;
  const std::size_t desired_start =
    nearest_index > index_distance ? nearest_index - index_distance : 0U;
  const std::size_t first_index = std::min(desired_start, maximum_start);
  const std::size_t middle_index = first_index + index_distance;
  const std::size_t last_index = middle_index + index_distance;
  return threePointCurvature(
    path[first_index].pose, path[middle_index].pose, path[last_index].pose);
}

double AdaptivePurePursuitCore::calculateLookaheadDistance(
  const double lateral_error, const double path_curvature, const double velocity,
  const double min_lookahead, PurePursuitOutput * output) const
{
  output->velocity_ld = std::abs(params_.ld_velocity_ratio * velocity);
  output->curvature_ld = -std::abs(params_.ld_curvature_ratio * path_curvature);
  output->lateral_error_ld = std::abs(lateral_error) >= params_.long_ld_lateral_error_threshold
    ? std::abs(params_.ld_lateral_error_ratio * lateral_error) : 0.0;
  return clampDouble(
    output->velocity_ld + output->curvature_ld + output->lateral_error_ld,
    min_lookahead, params_.max_lookahead_distance);
}

bool AdaptivePurePursuitCore::findLookaheadTarget(
  const std::vector<PathPoint> & path, const std::size_t nearest_index, const Pose2D & pose,
  const double lookahead_distance, Pose2D * target) const
{
  if (nearest_index >= path.size()) {
    return false;
  }
  const LaneDirection lane_direction = getLaneDirection(path);
  if (lane_direction == LaneDirection::kInvalid) {
    return false;
  }

  std::size_t next_index = path.size();
  for (std::size_t i = nearest_index; i < path.size(); ++i) {
    // This early return at the final point is intentional: it is the behavior of
    // PurePursuit::findNextPointIdx() in Autoware Universe.
    if (i + 1U == path.size()) {
      next_index = i;
      break;
    }
    const double point_relative_x = relativeX(pose, path[i].pose);
    if ((lane_direction == LaneDirection::kForward && point_relative_x < 0.0) ||
        (lane_direction == LaneDirection::kBackward && point_relative_x > 0.0)) {
      continue;
    }
    if (squaredDistance2d(pose, path[i].pose) > lookahead_distance * lookahead_distance) {
      next_index = i;
      break;
    }
  }
  if (next_index == path.size()) {
    return false;
  }
  if (next_index == 0U) {
    *target = path.front().pose;
    return true;
  }
  const PathPoint & first = path[next_index - 1U];
  const PathPoint & second = path[next_index];
  const double segment_x = second.pose.x - first.pose.x;
  const double segment_y = second.pose.y - first.pose.y;
  const double segment_length = std::hypot(segment_x, segment_y);
  if (segment_length < kEpsilon) {
    return false;
  }
  const double lateral_error = (segment_x * (pose.y - first.pose.y) -
    segment_y * (pose.x - first.pose.x)) / segment_length;
  const double unit_x = segment_x / segment_length;
  const double unit_y = segment_y / segment_length;
  const double foot_x = pose.x + lateral_error * unit_y;
  const double foot_y = pose.y - lateral_error * unit_x;
  // When the vehicle is farther from the path than the lookahead radius, the
  // lookahead circle and path segment do not intersect. Keep the controller
  // available for recovery by targeting the perpendicular projection instead
  // of rejecting the command.
  const double forward_distance = std::sqrt(std::max(
    0.0, lookahead_distance * lookahead_distance - lateral_error * lateral_error));
  // Waypoints are ordered in travel order.  As in Autoware's lerpNextTarget(), always
  // interpolate along that order; reverse travel is handled only while selecting the point.
  target->x = foot_x + forward_distance * unit_x;
  target->y = foot_y + forward_distance * unit_y;
  target->yaw = second.pose.yaw;
  return true;
}

double AdaptivePurePursuitCore::calculatePurePursuitCurvature(const Pose2D & pose, const Pose2D & target)
{
  const double relative_x = std::cos(pose.yaw) * (target.x - pose.x) +
                            std::sin(pose.yaw) * (target.y - pose.y);
  const double relative_y = -std::sin(pose.yaw) * (target.x - pose.x) +
                            std::cos(pose.yaw) * (target.y - pose.y);
  const double distance_squared = relative_x * relative_x + relative_y * relative_y;
  return distance_squared > kEpsilon ? 2.0 * relative_y / distance_squared : 1e9;
}

PurePursuitOutput AdaptivePurePursuitCore::failure(const std::string & reason) const
{
  PurePursuitOutput output;
  output.failure_reason = reason;
  output.used_previous_command = has_previous_command_;
  output.steering_tire_angle_rad = has_previous_command_ ? previous_steering_command_ : 0.0;
  return output;
}

PurePursuitOutput AdaptivePurePursuitCore::run(
  const std::vector<PathPoint> & path, const VehicleState & state)
{
  std::string parameter_reason;
  if (!parametersValid(&parameter_reason)) {
    return failure(parameter_reason);
  }
  if (!finitePose(state.pose) || !std::isfinite(state.velocity_mps) ||
      !std::isfinite(state.current_steering_tire_angle_rad)) {
    return failure("non-finite vehicle state");
  }
  if (path.size() < 3U) {
    return failure("trajectory has fewer than three points");
  }
  for (const auto & point : path) {
    if (!finitePose(point.pose) || !std::isfinite(point.target_velocity_mps)) {
      return failure("trajectory contains non-finite point");
    }
  }

  const auto resampled = resamplePath(path);
  if (resampled.size() < 3U) {
    return failure("resampled trajectory has fewer than three points");
  }
  std::size_t nearest_index = 0;
  if (!findNearestIndex(resampled, state.pose, &nearest_index)) {
    return failure("nearest trajectory point not found");
  }

  PurePursuitOutput output;
  output.lateral_error = calculateLateralError(resampled, state.pose);
  output.path_curvature = calculatePathCurvature(resampled, nearest_index);
  if (params_.lookahead_mode == LookaheadMode::kFixed) {
    output.lookahead_distance = params_.fixed_lookahead_distance;
  } else {
    const bool reverse = resampled[nearest_index].target_velocity_mps < 0.0;
    const double min_lookahead = reverse ? params_.reverse_min_lookahead_distance :
                                          params_.min_lookahead_distance;
    output.lookahead_distance = calculateLookaheadDistance(
      output.lateral_error, output.path_curvature, state.velocity_mps, min_lookahead, &output);
  }
  if (!findLookaheadTarget(
        resampled, nearest_index, state.pose, output.lookahead_distance, &output.target_point)) {
    return failure("lookahead target not found");
  }
  output.target_curvature = calculatePurePursuitCurvature(state.pose, output.target_point);
  if (!std::isfinite(output.target_curvature)) {
    return failure("non-finite target curvature");
  }
  output.steering_tire_angle_rad = clampDouble(
    std::atan(params_.wheel_base * output.target_curvature),
    -params_.max_steering_angle, params_.max_steering_angle);
  output.valid = std::isfinite(output.steering_tire_angle_rad);
  if (!output.valid) {
    return failure("non-finite steering command");
  }
  previous_steering_command_ = output.steering_tire_angle_rad;
  has_previous_command_ = true;
  return output;
}

}  // namespace trajectory_follower
