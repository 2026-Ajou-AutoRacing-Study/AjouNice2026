#include <autoware/behavior_path_avoidance_by_lane_change_module/factory.hpp>

#include "manager.hpp"

namespace autoware::behavior_path_planner
{

std::shared_ptr<SceneModuleManagerCore> makeInitializedAvoidanceByLaneChangeManager(
  ros::NodeHandle * node)
{
  if (!node) return {};
  auto manager = std::make_shared<AvoidanceByLaneChangeModuleManager>();
  manager->init(node);
  return manager;
}

}  // namespace autoware::behavior_path_planner
