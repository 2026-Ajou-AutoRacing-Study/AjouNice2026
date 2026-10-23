// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#include "lateral_collision_check.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <boost/geometry.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>

namespace autoware::motion_velocity_planner::lateral_collision
{
namespace
{
using Polygon = autoware_utils_geometry::Polygon2d;
constexpr double kEpsilon = 1e-6;
constexpr double kTimeStep = 0.1;

double lateral_extent(
  const autoware_perception_msgs::Shape & shape, double angle, double direction)
{
  using Shape = autoware_perception_msgs::Shape;
  if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  }
  if (shape.type == Shape::POLYGON) {
    double extent = 0.0;
    for (const auto & p : shape.footprint.points) {
      extent = std::max(extent, direction * (std::sin(angle) * p.x + std::cos(angle) * p.y));
    }
    return extent;
  }
  return std::abs(std::sin(angle)) * shape.dimensions.x / 2.0 +
         std::abs(std::cos(angle)) * shape.dimensions.y / 2.0;
}

double radius(const autoware_perception_msgs::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  }
  double result = std::hypot(shape.dimensions.x, shape.dimensions.y) / 2.0;
  for (const auto & p : shape.footprint.points) {
    result = std::max(result, std::hypot(static_cast<double>(p.x), static_cast<double>(p.y)));
  }
  return result;
}

Polygon swept(const Polygon & first, const Polygon & last)
{
  Polygon combined = first;
  Polygon hull;
  combined.outer().insert(combined.outer().end(), last.outer().begin(), last.outer().end());
  boost::geometry::convex_hull(combined, hull);
  boost::geometry::correct(hull);
  return hull;
}

std::optional<geometry_msgs::Pose> pose_at(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const std::vector<double> & arc, double s)
{
  if (s < 0.0 || s > arc.back() + kEpsilon) {
    return std::nullopt;
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), s);
  if (upper == arc.end()) return trajectory.back().pose;
  const std::size_t next = static_cast<std::size_t>(upper - arc.begin());
  if (next == 0) return trajectory.front().pose;
  const double length = arc[next] - arc[next - 1];
  if (length < kEpsilon) return trajectory[next].pose;
  return autoware_utils_geometry::calc_interpolated_pose(
    trajectory[next - 1].pose, trajectory[next].pose, (s - arc[next - 1]) / length, false);
}
}  // namespace

RelativeMotion relative_motion(
  const nav_msgs::Odometry & ego,
  const autoware_perception_msgs::PredictedObject & object,
  const geometry_msgs::Pose & current_object_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle, const double reference_yaw)
{
  RelativeMotion result;
  const auto & p = ego.pose.pose.position;
  const auto & o = current_object_pose.position;
  const double ego_angle = tf2::getYaw(ego.pose.pose.orientation) - reference_yaw;
  // The object's twist is expressed in its initial body frame, not the future pose frame.
  const double object_angle =
    tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation) - reference_yaw;
  const auto & ev = ego.twist.twist.linear;
  const auto & ov = object.kinematics.initial_twist_with_covariance.twist.linear;
  result.ego_lateral_velocity_mps = std::sin(ego_angle) * ev.x + std::cos(ego_angle) * ev.y;
  result.object_lateral_velocity_mps =
    std::sin(object_angle) * ov.x + std::cos(object_angle) * ov.y;
  result.relative_lateral_velocity_mps =
    result.object_lateral_velocity_mps - result.ego_lateral_velocity_mps;
  result.signed_distance_m =
    -(o.x - p.x) * std::sin(reference_yaw) + (o.y - p.y) * std::cos(reference_yaw);
  const double side = result.signed_distance_m >= 0.0 ? 1.0 : -1.0;
  // Project the asymmetric ego footprint toward the object, in the same normal.
  const double ego_extent =
    vehicle.vehicle_width_m / 2.0 * std::abs(std::cos(ego_angle)) +
    std::max(side * std::sin(ego_angle) * vehicle.max_longitudinal_offset_m,
             side * std::sin(ego_angle) * vehicle.min_longitudinal_offset_m);
  const double object_extent = lateral_extent(
    object.shape, tf2::getYaw(current_object_pose.orientation) - reference_yaw, -side);
  result.clearance_m =
    std::max(0.0, std::abs(result.signed_distance_m) - ego_extent - object_extent);
  result.closing_velocity_mps = -side * result.relative_lateral_velocity_mps;
  if (result.clearance_m <= kEpsilon) {
    result.time_to_contact_sec = 0.0;
  } else if (result.closing_velocity_mps > kEpsilon) {
    result.time_to_contact_sec = result.clearance_m / result.closing_velocity_mps;
  }
  return result;
}

bool passes_time_gate(
  const RelativeMotion & motion, const double margin_m, const double max_time_sec,
  const PredictionOverlap & prediction)
{
  return motion.clearance_m <= margin_m || motion.time_to_contact_sec <= max_time_sec ||
         prediction.collision;
}

std::vector<MotionSample> predict_longitudinal_motion(
  const double initial_speed_mps, const double initial_acceleration_mps2,
  const double target_acceleration_mps2, const double min_jerk_mps3,
  const double max_jerk_mps3, const double horizon_sec)
{
  if (
    !std::isfinite(initial_speed_mps) || initial_speed_mps < 0.0 ||
    !std::isfinite(initial_acceleration_mps2) || !std::isfinite(target_acceleration_mps2) ||
    !std::isfinite(min_jerk_mps3) || min_jerk_mps3 >= 0.0 ||
    !std::isfinite(max_jerk_mps3) || max_jerk_mps3 <= 0.0 ||
    !std::isfinite(horizon_sec) || horizon_sec <= 0.0) {
    return {};
  }
  MotionSample state{0.0, 0.0, initial_speed_mps, initial_acceleration_mps2};
  std::vector<MotionSample> samples{state};
  bool stopped = initial_speed_mps <= kEpsilon && initial_acceleration_mps2 < 0.0;
  while (state.time_sec < horizon_sec) {
    const double next_time = std::min(horizon_sec, state.time_sec + kTimeStep);
    double remaining = next_time - state.time_sec;
    while (!stopped && remaining > kEpsilon) {
      const double delta_a = target_acceleration_mps2 - state.acceleration_mps2;
      const double jerk = std::abs(delta_a) < kEpsilon ? 0.0
        : (delta_a < 0.0 ? min_jerk_mps3 : max_jerk_mps3);
      const double duration = jerk == 0.0 ? remaining
        : std::min(remaining, delta_a / jerk);
      const auto velocity_at = [&](double t) {
        return state.speed_mps + state.acceleration_mps2 * t + 0.5 * jerk * t * t;
      };
      // Catch a zero crossing even if positive jerk makes the end speed positive again.
      const double minimum_time = jerk > 0.0
        ? std::clamp(-state.acceleration_mps2 / jerk, 0.0, duration) : duration;
      double moving_duration = duration;
      if (velocity_at(minimum_time) < 0.0 || velocity_at(duration) <= 0.0) {
        double low = 0.0;
        double high = velocity_at(minimum_time) < 0.0 ? minimum_time : duration;
        for (int iteration = 0; iteration < 40; ++iteration) {
          const double mid = (low + high) / 2.0;
          if (velocity_at(mid) > 0.0) low = mid;
          else high = mid;
        }
        moving_duration = (low + high) / 2.0;
        stopped = true;
      }
      const double t = moving_duration;
      state.distance_m += std::max(
        0.0, state.speed_mps * t + 0.5 * state.acceleration_mps2 * t * t +
        jerk * t * t * t / 6.0);
      state.speed_mps = stopped ? 0.0 : std::max(0.0, velocity_at(t));
      state.acceleration_mps2 = stopped ? 0.0 : state.acceleration_mps2 + jerk * duration;
      remaining -= duration;
    }
    if (stopped) state.acceleration_mps2 = 0.0;
    state.time_sec = next_time;
    samples.push_back(state);
  }
  return samples;
}

PredictionOverlap predicted_overlap(
  const nav_msgs::Odometry & ego,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const autoware_perception_msgs::PredictedObject & object,
  const ros::Time & objects_stamp, const ros::Time & now,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const double horizon_sec, const std::size_t number_of_paths,
  const std::size_t ego_segment_index, const double pose_convergence_sec,
  const LongitudinalMotion & motion)
{
  PredictionOverlap result;
  if (
    trajectory.size() < 2 || ego_segment_index >= trajectory.size() - 1 ||
    !std::isfinite(horizon_sec) || horizon_sec <= 0.0 || number_of_paths == 0) {
    return result;
  }
  const double age = (now - objects_stamp).toSec();
  if (!std::isfinite(age)) return result;
  std::vector<double> arc(trajectory.size(), 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    arc[i] = arc[i - 1] + autoware_utils_geometry::calc_distance2d(
      trajectory[i - 1].pose.position, trajectory[i].pose.position);
  }
  if (arc.back() < kEpsilon) return result;
  const auto segment = ego_segment_index;
  const double offset = std::clamp(
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      trajectory, segment, ego.pose.pose.position),
    0.0, arc[segment + 1] - arc[segment]);
  const double start_s = arc[segment] + offset;
  const auto start_pose = pose_at(trajectory, arc, start_s);
  if (!start_pose) return result;
  const double speed = std::hypot(ego.twist.twist.linear.x, ego.twist.twist.linear.y);
  if (!std::isfinite(speed)) return result;

  struct MotionProfile
  {
    double target_acceleration;
    std::vector<MotionSample> samples;
  };
  // Index zero is ALWAYS the original constant-speed hypothesis. Additional
  // hypotheses may keep an obstacle, but cannot remove an existing risk.
  std::vector<MotionProfile> profiles{{0.0, {}}};
  if (
    std::isfinite(motion.min_acceleration_mps2) && motion.min_acceleration_mps2 <= 0.0 &&
    std::isfinite(motion.max_acceleration_mps2) && motion.max_acceleration_mps2 >= 0.0 &&
    std::isfinite(motion.initial_acceleration_mps2) &&
    std::isfinite(motion.min_jerk_mps3) && motion.min_jerk_mps3 < 0.0 &&
    std::isfinite(motion.max_jerk_mps3) && motion.max_jerk_mps3 > 0.0) {
    std::vector<double> targets;
    const auto add_target = [&](double target) {
      if (std::abs(target) < kEpsilon &&
          std::abs(motion.initial_acceleration_mps2) < kEpsilon) return;
      if (std::none_of(targets.begin(), targets.end(), [&](double a) {
        return std::abs(a - target) < kEpsilon;
      })) targets.push_back(target);
    };
    add_target(0.0);
    add_target(std::clamp(
      motion.initial_acceleration_mps2,
      motion.min_acceleration_mps2, motion.max_acceleration_mps2));
    // Include intermediate braking: constant speed and maximum braking can
    // both miss a collision that occurs with moderate deceleration.
    for (int i = 1; i <= 6; ++i) add_target(motion.min_acceleration_mps2 * i / 6.0);
    for (int i = 1; i <= 3; ++i) add_target(motion.max_acceleration_mps2 * i / 3.0);
    for (const double target : targets) {
      profiles.push_back({target, predict_longitudinal_motion(
        speed, motion.initial_acceleration_mps2, target,
        motion.min_jerk_mps3, motion.max_jerk_mps3, horizon_sec)});
    }
  }
  result.motion_hypotheses = profiles.size();
  const double yaw_error = autoware_utils_geometry::normalize_radian(
    tf2::getYaw(ego.pose.pose.orientation) - tf2::getYaw(start_pose->orientation));
  const auto ego_pose_at = [&](double t, const MotionProfile & profile)
    -> std::optional<geometry_msgs::Pose> {
    double distance = speed * t;
    if (!profile.samples.empty()) {
      const auto upper = std::upper_bound(
        profile.samples.begin(), profile.samples.end(), t,
        [](double time, const MotionSample & sample) { return time < sample.time_sec; });
      if (upper == profile.samples.begin()) distance = upper->distance_m;
      else if (upper == profile.samples.end()) distance = profile.samples.back().distance_m;
      else {
        const auto & previous = *(upper - 1);
        const double ratio = (t - previous.time_sec) / (upper->time_sec - previous.time_sec);
        distance = previous.distance_m + ratio * (upper->distance_m - previous.distance_m);
      }
    }
    auto pose = pose_at(trajectory, arc, start_s + distance);
    if (!pose) return std::nullopt;
    const double remaining = pose_convergence_sec > kEpsilon
      ? std::max(0.0, 1.0 - t / pose_convergence_sec) : (t == 0.0 ? 1.0 : 0.0);
    pose->position.x += (ego.pose.pose.position.x - start_pose->position.x) * remaining;
    pose->position.y += (ego.pose.pose.position.y - start_pose->position.y) * remaining;
    pose->orientation = autoware_utils_geometry::create_quaternion_from_yaw(
      tf2::getYaw(pose->orientation) + yaw_error * remaining);
    return pose;
  };
  std::vector<std::size_t> order(object.kinematics.predicted_paths.size());
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
    return object.kinematics.predicted_paths[a].confidence >
           object.kinematics.predicted_paths[b].confidence;
  });
  const double ego_radius = std::hypot(
    std::max(
      std::abs(vehicle.max_longitudinal_offset_m), std::abs(vehicle.min_longitudinal_offset_m)),
    vehicle.vehicle_width_m / 2.0);
  const double object_radius = radius(object.shape);
  for (std::size_t rank = 0; rank < std::min(number_of_paths, order.size()); ++rank) {
    const auto & path = object.kinematics.predicted_paths[order[rank]];
    if (path.path.size() < 2) continue;
    const double dt = path.time_step.sec + path.time_step.nanosec * 1e-9;
    if (!std::isfinite(dt) || dt <= 0.0) continue;
    const double begin = std::max(0.0, -age);
    const double end = std::min(horizon_sec, dt * (path.path.size() - 1) - age);
    if (begin > end) continue;
    for (std::size_t hypothesis = 0; hypothesis < profiles.size(); ++hypothesis) {
      const auto & profile = profiles[hypothesis];
      std::optional<geometry_msgs::Pose> previous_ego;
      boost::optional<geometry_msgs::Pose> previous_object;
      double previous_time = begin;
      // Include a final partial interval, and never extrapolate a prediction.
      for (double t = begin;; t = std::min(end, t + kTimeStep)) {
        const auto ep = ego_pose_at(t, profile);
        const auto op = autoware::object_recognition_utils::calcInterpolatedPose(
          path, std::max(0.0, age + t));
        if (!ep || !op) break;
        bool close = autoware_utils_geometry::calc_distance2d(ep->position, op->position) <=
                     ego_radius + object_radius;
        if (previous_ego && previous_object) {
          const double object_step = autoware_utils_geometry::calc_distance2d(
            previous_object->position, op->position);
          const double ego_step = autoware_utils_geometry::calc_distance2d(
            previous_ego->position, ep->position);
          close = close || autoware_utils_geometry::calc_distance2d(
            previous_ego->position, previous_object->position) <=
            ego_radius + object_radius + ego_step + object_step;
        }
        if (close) {
          auto ego_polygon = autoware_utils_geometry::to_footprint(
            *ep, vehicle.max_longitudinal_offset_m, -vehicle.min_longitudinal_offset_m,
            vehicle.vehicle_width_m);
          auto object_polygon = autoware_utils_geometry::to_polygon2d(*op, object.shape);
          if (previous_ego && previous_object) {
            ego_polygon = swept(
              ego_polygon, autoware_utils_geometry::to_footprint(
                *previous_ego, vehicle.max_longitudinal_offset_m,
                -vehicle.min_longitudinal_offset_m, vehicle.vehicle_width_m));
            object_polygon = swept(
              object_polygon, autoware_utils_geometry::to_polygon2d(*previous_object, object.shape));
          }
          if (boost::geometry::intersects(ego_polygon, object_polygon)) {
            const double contact_time = previous_ego ? previous_time : t;
            if (hypothesis == 0) result.constant_velocity_collision = true;
            if (!result.collision || contact_time < result.first_time_sec) {
              result.first_time_sec = contact_time;
              result.collision_target_acceleration_mps2 = hypothesis == 0
                ? std::numeric_limits<double>::quiet_NaN() : profile.target_acceleration;
            }
            result.collision = true;
            break;
          }
        }
        previous_ego = ep;
        previous_object = op;
        previous_time = t;
        if (t >= end) break;
      }
    }
  }
  return result;
}
}  // namespace autoware::motion_velocity_planner::lateral_collision
