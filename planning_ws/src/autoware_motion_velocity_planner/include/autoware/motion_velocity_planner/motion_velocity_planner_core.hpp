#pragma once

#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_planning_msgs/TrajectoryPoint.h>

#include <vector>

namespace autoware::motion_velocity_planner
{
class MotionVelocityPlannerCore
{
public:
  std::vector<autoware_planning_msgs::TrajectoryPoint> applyResults(
    std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory,
    const std::vector<VelocityPlanningResult> & results) const;
};
}  // namespace autoware::motion_velocity_planner
