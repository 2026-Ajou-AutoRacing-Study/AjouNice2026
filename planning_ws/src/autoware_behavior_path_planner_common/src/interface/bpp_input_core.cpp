#include <autoware/behavior_path_planner_common/interface/bpp_input_core.hpp>

#include <cmath>

namespace autoware::behavior_path_planner
{
bool BppInputCore::updateLateralOffset(
  const std::shared_ptr<PlannerData> & planner_data,
  const tier4_planning_msgs::LateralOffsetConstPtr & message)
{
  if (!planner_data || !message) return false;
  if (!planner_data->lateral_offset) {
    planner_data->lateral_offset = message;
    return true;
  }
  if (std::abs(planner_data->lateral_offset->lateral_offset - message->lateral_offset) < 1.0e-4) {
    return false;
  }
  planner_data->lateral_offset = message;
  return true;
}
}  // namespace autoware::behavior_path_planner
