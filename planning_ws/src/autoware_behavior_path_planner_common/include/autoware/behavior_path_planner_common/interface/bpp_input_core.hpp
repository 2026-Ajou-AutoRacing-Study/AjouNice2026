#pragma once

#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <memory>

namespace autoware::behavior_path_planner
{
// Source-equivalent BPP node ingress subset.  It owns only input acceptance;
// transport polling and planner execution remain the node integration layer.
class BppInputCore
{
public:
  static bool updateLateralOffset(
    const std::shared_ptr<PlannerData> & planner_data,
    const tier4_planning_msgs::LateralOffsetConstPtr & message);
};
}  // namespace autoware::behavior_path_planner
