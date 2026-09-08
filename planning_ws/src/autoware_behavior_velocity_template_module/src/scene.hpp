#pragma once

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>

namespace autoware::behavior_velocity_planner
{
class TemplateModule : public experimental::SceneModuleInterface
{
public:
  TemplateModule(
    lanelet::Id module_id, const ros::NodeHandle & node,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface> & planning_factor_interface);

  bool modifyPathVelocity(
    experimental::Trajectory & path, const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data) override;
  visualization_msgs::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;
};
}  // namespace autoware::behavior_velocity_planner
