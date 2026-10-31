// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_OBSTACLE_CRUISE_LANE_CHANGE_OWNERSHIP_HPP_
#define AUTOWARE_OBSTACLE_CRUISE_LANE_CHANGE_OWNERSHIP_HPP_
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <ros/time.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
// A fresh approved OUTPUT path is a lease, not a global LC-running boolean.
// Match the actual downstream geometry and speed envelope before handing
// off. Candidate topics and RTC safe flags never enter this predicate.
inline std::string lane_change_ownership_rejection(
  const autoware_internal_planning_msgs::PathWithLaneId & commitment,
  const ros::Time & received, const ros::Time & now,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Pose & ego)
{
  if (commitment.points.size() < 2) return "no_approved_lc_output";
  if (commitment.header.frame_id != "map") return "commitment_frame_mismatch";
  if (commitment.header.stamp.isZero() || now < commitment.header.stamp || now < received ||
      (now-commitment.header.stamp).toSec() > 0.3 || (now-received).toSec() > 0.3) {
    return "commitment_expired";
  }
  // A downstream reduction remains LC-owned and is checked by LC final-profile
  // feedback. Re-enabling ordinary following here would add a second speed
  // controller precisely when the constrained maneuver needs re-evaluation.
  if (trajectory.size() < 2) return "trajectory_missing";
  auto closest = std::min_element(trajectory.begin(), trajectory.end(), [&](const auto & a, const auto & b) {
    return std::hypot(a.pose.position.x-ego.position.x,a.pose.position.y-ego.position.y) <
           std::hypot(b.pose.position.x-ego.position.x,b.pose.position.y-ego.position.y);
  });
  if (std::hypot(closest->pose.position.x-ego.position.x,
                 closest->pose.position.y-ego.position.y) > 3.0) return "trajectory_off_ego";
  double length = 0.0;
  size_t checked = 0;
  for (auto it = closest; it != trajectory.end() && length <= 80.0; ++it) {
    if (it != closest) length += std::hypot(it->pose.position.x-(it-1)->pose.position.x,
                                          it->pose.position.y-(it-1)->pose.position.y);
    double distance = std::numeric_limits<double>::infinity(), speed = 0.0;
    for (size_t i = 1; i < commitment.points.size(); ++i) {
      const auto & a = commitment.points[i-1].point;
      const auto & b = commitment.points[i].point;
      const double dx = b.pose.position.x-a.pose.position.x, dy = b.pose.position.y-a.pose.position.y;
      const double l2 = dx*dx+dy*dy;
      if (l2 < 1e-9) continue;
      const double ratio = std::clamp(((it->pose.position.x-a.pose.position.x)*dx+
        (it->pose.position.y-a.pose.position.y)*dy)/l2,0.0,1.0);
      const double d = std::hypot(it->pose.position.x-a.pose.position.x-ratio*dx,
                                  it->pose.position.y-a.pose.position.y-ratio*dy);
      if (d < distance) {
        distance = d;
        speed = a.longitudinal_velocity_mps + ratio*(b.longitudinal_velocity_mps-a.longitudinal_velocity_mps);
      }
    }
    if (!std::isfinite(distance) || distance > 0.5) return "lc_geometry_changed";
    if (!std::isfinite(it->longitudinal_velocity_mps) || !std::isfinite(speed) ||
        it->longitudinal_velocity_mps < 0.0 || it->longitudinal_velocity_mps > speed + 0.5) return "lc_speed_changed";
    ++checked;
  }
  return checked >= 2 ? "" : "trajectory_too_short";
}
}
#endif
