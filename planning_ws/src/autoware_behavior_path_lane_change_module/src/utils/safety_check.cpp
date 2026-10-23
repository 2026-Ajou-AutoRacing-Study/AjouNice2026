// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/utils/safety_check.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::utils::lane_change
{
PhaseSafetyResult check_safety_by_phase(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::vector<safety::PoseWithVelocityStamped> & ego_prediction,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const safety::ExtendedPredictedObjects & objects, safety::CollisionCheckDebugMap & debug,
  const safety::RSSparams & prepare_params, const safety::RSSparams & execution_params,
  const double prepare_end_time, const double yaw_threshold, const bool collect_trace)
{
  PhaseSafetyResult result;
  if (std::isnan(prepare_end_time) || prepare_end_time < 0.0 || ego_prediction.empty()) {
    return result;
  }
  const double infinity = std::numeric_limits<double>::infinity();
  if (prepare_end_time > 0.0) {
    result.stage = "prepare";
    if (!safety::check_safety_with_rss_diagnostics(
          path, ego_prediction, vehicle, objects, debug, prepare_params, true, 1.0,
          yaw_threshold, collect_trace ? &result.trace : nullptr, 0.0, prepare_end_time)) {
      return result;
    }
  }
  if (std::isfinite(prepare_end_time)) {
    result.stage = "execution";
    if (!safety::check_safety_with_rss_diagnostics(
          path, ego_prediction, vehicle, objects, debug, execution_params, true, 1.0,
          yaw_threshold, collect_trace ? &result.trace : nullptr, prepare_end_time, infinity)) {
      return result;
    }
  }
  result.safe = true;
  return result;
}

std::optional<double> remaining_prepare_time(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::vector<safety::PoseWithVelocityStamped> & ego_prediction,
  const geometry_msgs::Pose & lane_changing_start)
{
  if (path.points.size() < 2 || ego_prediction.empty()) return std::nullopt;
  const auto remaining = [&](const auto & pose) {
    return autoware::motion_utils::calcSignedArcLength(
      path.points, pose.position, lane_changing_start.position);
  };
  double previous_distance = remaining(ego_prediction.front().pose);
  double previous_time = ego_prediction.front().time;
  if (!std::isfinite(previous_distance) || previous_time != 0.0) return std::nullopt;
  if (previous_distance <= 0.0) return 0.0;
  for (size_t i = 1; i < ego_prediction.size(); ++i) {
    const auto & point = ego_prediction[i];
    const double distance = remaining(point.pose);
    if (!std::isfinite(distance) || !std::isfinite(point.time) || point.time <= previous_time) {
      return std::nullopt;
    }
    if (distance <= 0.0) {
      const double ratio = previous_distance / (previous_distance - distance);
      return previous_time + ratio * (point.time - previous_time);
    }
    previous_distance = distance;
    previous_time = point.time;
  }
  return std::numeric_limits<double>::infinity();
}
}
