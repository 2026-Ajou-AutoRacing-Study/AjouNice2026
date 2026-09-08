#include "experimental_adapter.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace autoware::behavior_velocity_planner::experimental
{
namespace
{
autoware_internal_planning_msgs::PathWithLaneId asPath(const Trajectory & trajectory)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.points = trajectory.restore();
  return path;
}

bool restoreTrajectory(
  const autoware_internal_planning_msgs::PathWithLaneId & path_message, Trajectory * trajectory)
{
  const auto rebuilt = Trajectory::Builder{}.build(path_message.points);
  if (!rebuilt) {
    ROS_ERROR("experimental stop-line adapter could not rebuild the output trajectory");
    return false;
  }
  *trajectory = rebuilt.value();
  return true;
}
}  // namespace

void StopLineModulePlugin::init(ros::NodeHandle & node) { legacy_plugin_.init(node); }

RequiredSubscriptionInfo StopLineModulePlugin::getRequiredSubscriptions()
{
  return legacy_plugin_.getRequiredSubscriptions();
}

void StopLineModulePlugin::plan(
  Trajectory & path, const std_msgs::Header &, const std::vector<geometry_msgs::Point> &,
  const std::vector<geometry_msgs::Point> &, const PlannerData &)
{
  auto path_message = asPath(path);
  legacy_plugin_.plan(&path_message);
  restoreTrajectory(path_message, &path);
}

void StopLineModulePlugin::updateSceneModuleInstances(
  const Trajectory & path, const ros::Time &, const PlannerData & planner_data)
{
  const auto path_message = asPath(path);
  legacy_plugin_.updateSceneModuleInstances(std::make_shared<PlannerData>(planner_data), path_message);
}

const char * StopLineModulePlugin::getModuleName() { return legacy_plugin_.getModuleName(); }
}  // namespace autoware::behavior_velocity_planner::experimental

PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::StopLineModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
