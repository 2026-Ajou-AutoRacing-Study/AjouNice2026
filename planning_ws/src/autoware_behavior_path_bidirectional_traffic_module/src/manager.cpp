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
#include "autoware/behavior_path_bidirectional_traffic_module/manager.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/scene.hpp"

#include <ros/ros.h>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{

bool BidirectionalTrafficModuleManager::init(ros::NodeHandle * node)
{
  if (!node) return false;
  node_ = node;
  if (!initProviders({})) return false;
  parameters_ = std::make_shared<BidirectionalTrafficModuleParameters>();
  return parameters_->init_from_node(node, "bidirectional_traffic");
}

std::unique_ptr<SceneModuleCore>
BidirectionalTrafficModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<BidirectionalTrafficModule>(
    name(), parameters_, rtc_interfaces_, marker_interfaces_, planning_factor_interface_);
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::BidirectionalTrafficModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
