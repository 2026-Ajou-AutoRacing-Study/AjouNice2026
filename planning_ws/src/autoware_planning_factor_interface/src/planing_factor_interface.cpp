// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>

namespace autoware
{
namespace planning_factor_interface
{
template void PlanningFactorInterface::add<autoware_internal_planning_msgs::PathPointWithLaneId>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &, const Pose &,
  const Pose &, const uint16_t, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::PathPoint>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const Pose &, const Pose &,
  const uint16_t, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const Pose &, const Pose &,
  const uint16_t, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
template void PlanningFactorInterface::add<autoware_internal_planning_msgs::PathPointWithLaneId>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &, const Pose &,
  const Pose &, const Pose &, const uint16_t, const SafetyFactorArray &, const bool,
  const double, const double, const double, const double, const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::PathPoint>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const Pose &, const Pose &, const Pose &,
  const uint16_t, const SafetyFactorArray &, const bool, const double, const double,
  const double, const double, const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const Pose &, const Pose &,
  const Pose &, const uint16_t, const SafetyFactorArray &, const bool, const double,
  const double, const double, const double, const std::string &);
}  // namespace planning_factor_interface
}  // namespace autoware
