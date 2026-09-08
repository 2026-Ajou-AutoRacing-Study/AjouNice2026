#include "scene.hpp"

namespace autoware::behavior_velocity_planner
{
TemplateModule::TemplateModule(
  const lanelet::Id module_id, const ros::NodeHandle & node,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface(module_id, node, time_keeper, planning_factor_interface)
{
}

visualization_msgs::MarkerArray TemplateModule::createDebugMarkerArray() { return {}; }
autoware::motion_utils::VirtualWalls TemplateModule::createVirtualWalls() { return {}; }

bool TemplateModule::modifyPathVelocity(
  [[maybe_unused]] experimental::Trajectory & path,
  [[maybe_unused]] const std::vector<geometry_msgs::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::Point> & right_bound,
  [[maybe_unused]] const PlannerData & planner_data)
{
  ROS_INFO_STREAM_NAMED(node_.getNamespace(), "Template Module is executing!");
  return false;
}
}  // namespace autoware::behavior_velocity_planner
