#include <autoware/behavior_path_goal_planner_module/freespace_bpp_adapter.hpp>

#include <autoware/behavior_path_goal_planner_module/pull_over_lane_adapter.hpp>
#include <autoware/route_handler/route_handler.hpp>

namespace autoware::behavior_path_planner
{
FreespaceBppAdapter::FreespaceBppAdapter(
  const ros::NodeHandle & node,
  const autoware::freespace_planning_algorithms::VehicleShape & vehicle_shape,
  const double velocity, const double acceleration)
: core_(node, vehicle_shape), velocity_(velocity), acceleration_(acceleration)
{
}

std::optional<BehaviorModuleOutput> FreespaceBppAdapter::plan(
  const std::shared_ptr<const PlannerData> & planner_data,
  const geometry_msgs::Pose & end_pose, const geometry_msgs::Pose & goal_pose,
  const size_t goal_id, const bool use_back)
{
  if (!planner_data || !planner_data->route_handler || !planner_data->self_odometry ||
      !planner_data->costmap || !planner_data->dynamic_object || velocity_ <= 0.0 ||
      !planner_data->route_handler->isHandlerReady() || !planner_data->route_handler->isMapMsgReady() ||
      !planner_data->dynamic_object->objects.empty()) {
    return std::nullopt;
  }
  const auto lanes = selectPullOverLanes(planner_data->route_handler, goal_pose, false, 0.0, 0.0);
  if (!lanes || lanes->empty()) return std::nullopt;
  core_.setMap(*planner_data->costmap);
  const auto pull_over = core_.planCompletedPullOverPath(
    planner_data->self_odometry->pose.pose, end_pose, goal_pose, velocity_, *lanes, goal_id,
    use_back, acceleration_);
  if (!pull_over) return std::nullopt;
  BehaviorModuleOutput output;
  output.path = pull_over->fullPath();
  output.reference_path = output.path;
  return output.path.points.empty() ? std::nullopt : std::optional<BehaviorModuleOutput>{output};
}
}  // namespace autoware::behavior_path_planner
