// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_dynamic_obstacle_avoidance_module/scene.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"

#include "autoware/behavior_path_dynamic_obstacle_avoidance_module/ros1_compat.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

class DynamicObstacleAvoidanceModuleManager : public SceneModuleManagerInterface
{
public:
  DynamicObstacleAvoidanceModuleManager()
  : SceneModuleManagerInterface{"dynamic_obstacle_avoidance"}
  {
  }

  bool init(ros::NodeHandle * node);
  bool initialize(ros::NodeHandle * node) override { return init(node); }

  std::unique_ptr<SceneModuleCore> createNewSceneModuleInstance() override
  {
    return std::make_unique<DynamicObstacleAvoidanceModule>(
      name(), *node_, parameters_, rtc_interfaces_, marker_interfaces_, planning_factor_interface_);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters);

private:
  std::shared_ptr<DynamicAvoidanceParameters> parameters_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__MANAGER_HPP_
