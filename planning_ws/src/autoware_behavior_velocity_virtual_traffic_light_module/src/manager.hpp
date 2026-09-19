// Copyright 2021 Tier IV, Inc.
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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_manager_interface.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <tier4_v2x_msgs/InfrastructureCommandArray.h>
#include <tier4_v2x_msgs/VirtualTrafficLightStateArray.h>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class VirtualTrafficLightModuleManager
: public SceneModuleManagerInterface<VirtualTrafficLightModule>
{
public:
  explicit VirtualTrafficLightModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "virtual_traffic_light"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

  void plan(autoware_internal_planning_msgs::PathWithLaneId * path) override;

private:
  VirtualTrafficLightModule::PlannerParam planner_param_;

  void launchNewModules(const autoware_internal_planning_msgs::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<VirtualTrafficLightModule> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::PathWithLaneId & path) override;

  ros::Subscriber sub_virtual_traffic_light_states_;
  ros::Publisher pub_infrastructure_commands_;
  tier4_v2x_msgs::VirtualTrafficLightStateArrayConstPtr virtual_traffic_light_states_;
  void onVirtualTrafficLightStates(const tier4_v2x_msgs::VirtualTrafficLightStateArrayConstPtr & message)
  {
    virtual_traffic_light_states_ = message;
  }
};

class VirtualTrafficLightModulePlugin : public PluginWrapper<VirtualTrafficLightModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
