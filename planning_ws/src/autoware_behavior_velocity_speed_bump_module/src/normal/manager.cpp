#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>

#include <pluginlib/class_list_macros.h>

namespace autoware::behavior_velocity_planner
{
using lanelet::autoware::SpeedBump;

namespace
{
template <class T>
T require(ros::NodeHandle & node, const std::string & name)
{
  T value{};
  if (!node.getParam(name, value)) throw std::runtime_error("missing parameter: " + name);
  return value;
}
}  // namespace

SpeedBumpModuleManager::SpeedBumpModuleManager(ros::NodeHandle & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.slow_start_margin = require<double>(node, ns + "/slow_start_margin");
  planner_param_.slow_end_margin = require<double>(node, ns + "/slow_end_margin");
  planner_param_.print_debug_info = require<bool>(node, ns + "/print_debug_info");
  const std::string calc = ns + "/speed_calculation";
  planner_param_.speed_calculation_min_height =
    static_cast<float>(require<double>(node, calc + "/min_height"));
  planner_param_.speed_calculation_max_height =
    static_cast<float>(require<double>(node, calc + "/max_height"));
  planner_param_.speed_calculation_min_speed =
    static_cast<float>(require<double>(node, calc + "/min_speed"));
  planner_param_.speed_calculation_max_speed =
    static_cast<float>(require<double>(node, calc + "/max_speed"));
}

void SpeedBumpModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  for (const auto & speed_bump_with_lane_id : planning_utils::getRegElemMapOnPath<SpeedBump>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto lane_id = speed_bump_with_lane_id.second.id();
    const auto module_id = speed_bump_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<SpeedBumpModule>(
        module_id, lane_id, *speed_bump_with_lane_id.first, planner_param_, *node_, time_keeper_,
        planning_factor_interface_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SpeedBumpModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  const auto ids = planning_utils::getRegElemIdSetOnPath<SpeedBump>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);
  return [ids](const std::shared_ptr<SceneModuleInterface> & module) {
    return ids.count(module->getModuleId()) == 0;
  };
}
}  // namespace autoware::behavior_velocity_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::SpeedBumpModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
