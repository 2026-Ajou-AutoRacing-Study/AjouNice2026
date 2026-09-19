#pragma once

#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/VelocityLimitClearCommand.h>
#include <geometry_msgs/Point.h>

#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct SlowdownInterval
{
  SlowdownInterval(const geometry_msgs::Point & from_, const geometry_msgs::Point & to_, double velocity_)
  : from(from_), to(to_), velocity(velocity_) {}
  geometry_msgs::Point from{};
  geometry_msgs::Point to{};
  double velocity{};
};

struct VelocityPlanningResult
{
  std::vector<geometry_msgs::Point> stop_points{};
  std::vector<SlowdownInterval> slowdown_intervals{};
  std::optional<autoware_internal_planning_msgs::VelocityLimit> velocity_limit{};
  std::optional<autoware_internal_planning_msgs::VelocityLimitClearCommand> velocity_limit_clear_command{};
};
}  // namespace autoware::motion_velocity_planner
