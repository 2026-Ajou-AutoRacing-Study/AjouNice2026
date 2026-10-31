// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>

#include <optional>

namespace autoware::behavior_path_planner::utils::lane_change
{
namespace safety = path_safety_checker;

struct PhaseSafetyResult
{
  bool safe{false};
  const char * stage{"invalid_phase"};
  safety::RSSCollisionTrace trace;
};

// Prediction time is relative to this planning cycle. Preparation owns [0,T),
// execution owns [T,infinity). Actual body overlap is checked in both phases.
PhaseSafetyResult check_safety_by_phase(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::vector<safety::PoseWithVelocityStamped> & ego_prediction,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const safety::ExtendedPredictedObjects & objects, safety::CollisionCheckDebugMap & debug,
  const safety::RSSparams & prepare_params, const safety::RSSparams & execution_params,
  double prepare_end_time, double yaw_threshold, bool collect_trace = false);

// An approved maneuver's original preparation duration does not start over on
// every cycle. Locate its start on each new ego-motion hypothesis instead.
// Infinity means that this hypothesis never reaches the execution phase.
std::optional<double> remaining_prepare_time(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::vector<safety::PoseWithVelocityStamped> & ego_prediction,
  const geometry_msgs::Pose & lane_changing_start);
}
