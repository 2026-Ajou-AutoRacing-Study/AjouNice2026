#pragma once

#include <autoware/behavior_path_planner_common/turn_signal_types.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>

namespace autoware::behavior_path_planner
{

// Route/lanelet discovery remains in TurnSignalDecider.  This ported slice is
// only the deterministic arbitration of already-discovered intersection,
// roundabout and behavior candidates.
class TurnSignalResolver
{
public:
  explicit TurnSignalResolver(double base_link2front = 0.0) : base_link2front_(base_link2front) {}

  autoware_vehicle_msgs::TurnIndicatorsCommand resolve(
    const autoware_internal_planning_msgs::PathWithLaneId & path,
    const geometry_msgs::Pose & current_pose, size_t current_segment_index,
    const TurnSignalInfo & intersection_signal_info,
    const TurnSignalInfo & roundabout_signal_info,
    const TurnSignalInfo & behavior_signal_info,
    double nearest_dist_threshold, double nearest_yaw_threshold) const;

private:
  double base_link2front_{0.0};
};

}  // namespace autoware::behavior_path_planner
