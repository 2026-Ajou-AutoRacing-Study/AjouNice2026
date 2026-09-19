#include "autoware/behavior_velocity_rtc_interface/experimental/scene_module_interface_with_rtc.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

namespace autoware::behavior_velocity_planner::experimental
{
SceneModuleInterfaceWithRTC::SceneModuleInterfaceWithRTC(int64_t module_id, const ros::NodeHandle & node,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface(module_id, node, std::move(time_keeper), std::move(planning_factor_interface)) {}

SceneModuleManagerInterfaceWithRTC::SceneModuleManagerInterfaceWithRTC(
  ros::NodeHandle & node, const char * module_name, bool enable_rtc)
: SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>(node, module_name),
  rtc_interface_(&node, module_name, enable_rtc), objects_marker_(&node, module_name) {}

void SceneModuleManagerInterfaceWithRTC::plan(Trajectory & path, const std_msgs::Header & header,
  const std::vector<geometry_msgs::Point> & left_bound, const std::vector<geometry_msgs::Point> & right_bound,
  const PlannerData & planner_data)
{
  setActivation();
  SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::plan(path, header, left_bound, right_bound, planner_data);
  sendRTC(header.stamp);
  for (const auto & module : scene_modules_) {
    for (const auto & object : module->getObjectsOfInterestData()) objects_marker_.insertObjectData(object.pose, object.shape, object.color);
    module->clearObjectsOfInterestData();
  }
  objects_marker_.publishMarkerArray();
}

void SceneModuleManagerInterfaceWithRTC::setActivation()
{
  for (const auto & module : scene_modules_) {
    const auto uuid = getUUID(module->getModuleId());
    module->setActivation(rtc_interface_.isActivated(uuid));
    module->setRTCEnabled(rtc_interface_.isRTCEnabled(uuid));
  }
}
void SceneModuleManagerInterfaceWithRTC::sendRTC(const ros::Time & stamp)
{
  for (const auto & module : scene_modules_) {
    const auto uuid = getUUID(module->getModuleId());
    const auto state = !module->isActivated() && module->isSafe() ? State::WAITING_FOR_EXECUTION : State::RUNNING;
    rtc_interface_.updateCooperateStatus(uuid, module->isSafe(), state, module->getDistance(), module->getDistance(), stamp);
  }
  rtc_interface_.removeExpiredCooperateStatus();
  rtc_interface_.publishCooperateStatus(stamp);
}
UUID SceneModuleManagerInterfaceWithRTC::getUUID(int64_t module_id) const
{ const auto it = map_uuid_.find(module_id); return it == map_uuid_.end() ? UUID{} : it->second; }
void SceneModuleManagerInterfaceWithRTC::generateUUID(int64_t module_id) { map_uuid_.emplace(module_id, autoware_utils::generate_uuid()); }
void SceneModuleManagerInterfaceWithRTC::removeUUID(int64_t module_id) { map_uuid_.erase(module_id); }
void SceneModuleManagerInterfaceWithRTC::deleteExpiredModules(const Trajectory & path, const PlannerData & planner_data)
{
  const auto expired = getModuleExpiredFunction(path, planner_data);
  for (auto it = scene_modules_.begin(); it != scene_modules_.end();) {
    if (expired(*it)) {
      const auto uuid = getUUID((*it)->getModuleId());
      rtc_interface_.updateCooperateStatus(uuid, (*it)->isSafe(), State::SUCCEEDED,
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), ros::Time::now());
      removeUUID((*it)->getModuleId()); registered_module_id_set_.erase((*it)->getModuleId()); it = scene_modules_.erase(it);
    } else ++it;
  }
}
}  // namespace autoware::behavior_velocity_planner::experimental
