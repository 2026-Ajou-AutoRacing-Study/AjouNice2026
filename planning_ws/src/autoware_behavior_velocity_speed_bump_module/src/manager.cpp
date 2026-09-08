#include "manager.hpp"
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace autoware::behavior_velocity_planner::experimental
{
using lanelet::autoware::SpeedBump;
SpeedBumpModuleManager::SpeedBumpModuleManager(ros::NodeHandle & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  node.param(ns + "/slow_start_margin", param_.slow_start_margin, 1.0);
  node.param(ns + "/slow_end_margin", param_.slow_end_margin, 1.0);
  node.param(ns + "/print_debug_info", param_.print_debug_info, false);
  node.param(ns + "/speed_calculation/min_height", param_.min_height, 0.0F);
  node.param(ns + "/speed_calculation/max_height", param_.max_height, 0.2F);
  node.param(ns + "/speed_calculation/min_speed", param_.min_speed, 1.0F);
  node.param(ns + "/speed_calculation/max_speed", param_.max_speed, 5.0F);
}
void SpeedBumpModuleManager::launchNewModules(const Trajectory & path, const ros::Time &,
  const PlannerData & data)
{
  if (!data.route_handler_ || !data.current_odometry) return;
  autoware_internal_planning_msgs::PathWithLaneId message; message.points = path.restore();
  for (const auto & element_and_lane : planning_utils::getRegElemMapOnPath<SpeedBump>(
         message, data.route_handler_->getLaneletMapPtr(), data.current_odometry->pose)) {
    const auto id = element_and_lane.first->id();
    if (!isModuleRegistered(id)) registerModule(std::make_shared<SpeedBumpModule>(id,
      *element_and_lane.first, param_, node_, time_keeper_, planning_factor_interface_), data);
  }
}
std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SpeedBumpModuleManager::getModuleExpiredFunction(const Trajectory & path, const PlannerData & data)
{
  autoware_internal_planning_msgs::PathWithLaneId message; message.points = path.restore();
  const auto ids = (!data.route_handler_ || !data.current_odometry) ? std::set<int64_t>{} :
    planning_utils::getRegElemIdSetOnPath<SpeedBump>(message, data.route_handler_->getLaneletMapPtr(),
      data.current_odometry->pose);
  return [ids](const std::shared_ptr<SceneModuleInterface> & module) {
    return ids.count(module->getModuleId()) == 0;
  };
}
}  // namespace autoware::behavior_velocity_planner::experimental
PLUGINLIB_EXPORT_CLASS(autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
