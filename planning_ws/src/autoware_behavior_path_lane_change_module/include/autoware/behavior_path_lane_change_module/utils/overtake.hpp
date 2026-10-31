// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <autoware/behavior_path_lane_change_module/structs/parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>

#include <optional>
#include <string>

namespace autoware::behavior_path_planner::utils::lane_change
{
namespace safety = path_safety_checker;

// Boundary-to-boundary gap projected on a common road reference. Positive
// means the complete object is behind ego, including yawed/truck footprints.
double rear_body_gap(
  const autoware_internal_planning_msgs::PathWithLaneId & reference,
  const geometry_msgs::Pose & ego, const autoware_utils_geometry::Polygon2d & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle);

double cooperative_rear_distance(
  double ego_speed, double rear_speed,
  const autoware::behavior_path_planner::lane_change::OvertakeParameters & parameters);

enum class MergeSlot { Ahead, Behind, Unavailable };
// Slot classification never substitutes for the complete collision check.
// Do not classify a short/ambiguous prediction as an ordinary rear candidate.
MergeSlot predicted_merge_slot(
  const autoware_internal_planning_msgs::PathWithLaneId & reference,
  const geometry_msgs::Pose & merge_end, double maneuver_time,
  const safety::ExtendedPredictedObject & target,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle);

bool merges_ahead_of_target(
  const autoware_internal_planning_msgs::PathWithLaneId & reference,
  const geometry_msgs::Pose & merge_end, double maneuver_time,
  const safety::ExtendedPredictedObject & target,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle);

// Constant-motion object predictions remain in the check. The smaller margin
// assumes nominal ego motion plus bounded NPC response, not an ego emergency
// stop. Actual-motion and downstream-speed hypotheses must also pass.
bool check_cooperative_rear(
  const autoware_internal_planning_msgs::PathWithLaneId & reference,
  const std::vector<safety::PoseWithVelocityStamped> & ego,
  const safety::ExtendedPredictedObject & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const autoware::behavior_path_planner::lane_change::OvertakeParameters & parameters,
  double lateral_margin, safety::CollisionCheckDebugMap & debug,
  safety::RSSCollisionTrace * trace = nullptr);
}
