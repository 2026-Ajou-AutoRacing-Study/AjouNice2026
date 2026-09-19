#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_manager_core.hpp>

#include <ros/ros.h>

#include <memory>

namespace autoware::behavior_path_planner
{

// Narrow provider ABI for the BPP execution core.  It exposes lifecycle-ready
// manager ownership without making BPP depend on this module's private scene,
// parameter, or ROS2-compatibility headers.
std::shared_ptr<SceneModuleManagerCore> makeInitializedAvoidanceByLaneChangeManager(
  ros::NodeHandle * node);

}  // namespace autoware::behavior_path_planner
