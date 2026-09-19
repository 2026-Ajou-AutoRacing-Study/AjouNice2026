#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <limits>

namespace autoware::behavior_path_planner
{
SceneModuleManagerInterface::SceneModuleManagerInterface(std::string name, ros::NodeHandle * node)
: SceneModuleManagerRos1(node, std::move(name))
{
}

bool SceneModuleManagerInterface::init(
  ros::NodeHandle * node, const std::vector<std::string> & rtc_types, const bool enable_rtc)
{
  node_ = node;
  if (!initInterface(rtc_types, enable_rtc)) return false;
  virtual_wall_publisher_ = node_->advertise<visualization_msgs::MarkerArray>("virtual_wall/" + name(), 20);
  return true;
}
void SceneModuleManagerInterface::publishVirtualWall() const
{
  visualization_msgs::MarkerArray markers;
  const uint32_t marker_id = std::numeric_limits<uint8_t>::max();
  for (const auto & core : activeModules()) {
    const auto module = std::dynamic_pointer_cast<SceneModuleInterface>(core);
    if (!module) continue;
    const auto append = [&](const auto & values) {
      markers.markers.insert(markers.markers.end(), values.markers.begin(), values.markers.end());
    };
    const auto add_wall = [&](const PoseWithDetailOpt & pose, const auto & create) {
      if (!pose) return;
      const auto text = module->name() + (pose->detail.empty() ? "" : " \n(" + pose->detail + ")");
      append(create(pose->pose, text, ros::Time::now(), marker_id, 0.0, "", true));
    };
    add_wall(module->getStopPose(), autoware::motion_utils::createStopVirtualWallMarker);
    add_wall(module->getSlowPose(), autoware::motion_utils::createSlowDownVirtualWallMarker);
    add_wall(module->getDeadPose(), autoware::motion_utils::createDeadLineVirtualWallMarker);
    append(module->getModuleVirtualWall());
    module->resetWallPoses();
  }
  if (virtual_wall_publisher_) virtual_wall_publisher_.publish(markers);
}
}  // namespace autoware::behavior_path_planner
