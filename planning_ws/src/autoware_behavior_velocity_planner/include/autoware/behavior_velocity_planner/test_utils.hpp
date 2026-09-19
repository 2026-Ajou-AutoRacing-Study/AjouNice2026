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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__TEST_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__TEST_UTILS_HPP_

#include "full_node.hpp"

#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::behavior_velocity_planner::FullBehaviorVelocityPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

struct PluginInfo
{
  std::string module_name;  // e.g. crosswalk
  std::string plugin_name;  // e.g. autoware::behavior_velocity_planner::CrosswalkModulePlugin
};

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager();

std::shared_ptr<FullBehaviorVelocityPlannerNode> generateNode(
  const std::vector<PluginInfo> & plugin_info_vec);

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<FullBehaviorVelocityPlannerNode> test_target_node);
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__TEST_UTILS_HPP_
