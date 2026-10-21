// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#ifndef OBSTACLE_CRUISE_LATERAL_COLLISION_CHECK_HPP_
#define OBSTACLE_CRUISE_LATERAL_COLLISION_CHECK_HPP_

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

#include <cstddef>
#include <limits>
#include <vector>

namespace autoware::motion_velocity_planner::lateral_collision
{
struct RelativeMotion
{
  double signed_distance_m{};
  double clearance_m{};
  double object_lateral_velocity_mps{};
  double ego_lateral_velocity_mps{};
  double relative_lateral_velocity_mps{};
  double closing_velocity_mps{};
  double time_to_contact_sec{std::numeric_limits<double>::infinity()};
};

// Both body-frame twists are rotated to the map frame, then projected onto the
// SAME reference normal used for signed center distance and shape clearance.
RelativeMotion relative_motion(
  const nav_msgs::Odometry & ego,
  const autoware_perception_msgs::PredictedObject & object,
  const geometry_msgs::Pose & current_object_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  double reference_yaw);

struct PredictionOverlap
{
  bool collision{};
  double first_time_sec{std::numeric_limits<double>::infinity()};
  bool constant_velocity_collision{};
  std::size_t motion_hypotheses{1};
  double collision_target_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
};

struct LongitudinalMotion
{
  double initial_acceleration_mps2{};
  double min_acceleration_mps2{};
  double max_acceleration_mps2{};
  double min_jerk_mps3{-10.0};
  double max_jerk_mps3{100.0};
};

struct MotionSample
{
  double time_sec{};
  double distance_m{};
  double speed_mps{};
  double acceleration_mps2{};
};

// Integrate from the measured state to a target acceleration within jerk limits.
// Braking stops at zero speed without moving backward. Includes the last partial interval.
std::vector<MotionSample> predict_longitudinal_motion(
  double initial_speed_mps, double initial_acceleration_mps2, double target_acceleration_mps2,
  double min_jerk_mps3, double max_jerk_mps3, double horizon_sec);

// Always retain measured constant speed, plus jerk-limited acceleration hypotheses.
// A collision in ANY hypothesis keeps the object eligible for precise checks.
// Compare swept footprints over matching 0.1 s intervals.
// Object predictions are shifted by their header age; never extrapolate them.
PredictionOverlap predicted_overlap(
  const nav_msgs::Odometry & ego,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const autoware_perception_msgs::PredictedObject & object,
  const ros::Time & objects_stamp, const ros::Time & now,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  double horizon_sec, std::size_t number_of_paths, std::size_t ego_segment_index,
  double pose_convergence_sec, const LongitudinalMotion & motion = {});

bool passes_time_gate(
  const RelativeMotion & motion, double margin_m, double max_time_sec,
  const PredictionOverlap & prediction);
}  // namespace autoware::motion_velocity_planner::lateral_collision
#endif
