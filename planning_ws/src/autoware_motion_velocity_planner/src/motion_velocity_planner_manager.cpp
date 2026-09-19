#include <autoware/motion_velocity_planner/motion_velocity_planner_manager.hpp>

namespace autoware::motion_velocity_planner
{
MotionVelocityPlannerManager::MotionVelocityPlannerManager()
: plugin_loader_("autoware_motion_velocity_planner_common", "autoware::motion_velocity_planner::PluginModuleInterface")
{
}

bool MotionVelocityPlannerManager::launchModule(ros::NodeHandle & node, const std::string & name)
{
  if (!plugin_loader_.isClassAvailable(name)) {
    ROS_ERROR_STREAM_NAMED(node.getNamespace(), "The motion-velocity plugin '" << name << "' is not available.");
    return false;
  }
  ROS_INFO_STREAM_NAMED(
    node.getNamespace(), "The motion-velocity plugin '" << name << "' resolves to '"
                                                       << plugin_loader_.getClassLibraryPath(name) << "'.");
  const auto module = std::shared_ptr<PluginModuleInterface>(plugin_loader_.createUniqueInstance(name));
  if (!core_.registerModule(node, name, module)) {
    ROS_WARN_STREAM_NAMED(node.getNamespace(), "The motion-velocity plugin '" << name << "' is already loaded.");
    return false;
  }
  ROS_INFO_STREAM_NAMED(node.getNamespace(), "The motion-velocity plugin '" << name << "' is loaded.");
  return true;
}
}  // namespace autoware::motion_velocity_planner
