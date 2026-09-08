// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "autoware/behavior_path_sampling_planner_module/sampling_planner_module.hpp"
#include "autoware/behavior_path_sampling_planner_module/sampling_planner_parameters.hpp"

#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
class SamplingPlannerModuleManager : public SceneModuleManagerInterface
{
public:
  SamplingPlannerModuleManager() : SceneModuleManagerInterface{"sampling_planner"} {}
  void init(ros::NodeHandle * node);
  bool initialize(ros::NodeHandle * node) override;
  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

  std::unique_ptr<SceneModuleCore> createNewSceneModuleInstance() override
  {
    return std::make_unique<SamplingPlannerModule>(
      name(), *node_, parameters_, rtc_interfaces_, marker_interfaces_, planning_factor_interface_);
  }

  void reloadParameters();

private:
  std::shared_ptr<SamplingPlannerParameters> parameters_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__MANAGER_HPP_
