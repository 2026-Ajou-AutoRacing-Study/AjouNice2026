// Copyright 2025 Tier IV, Inc.
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

#ifndef EXPERIMENTAL__MANAGER_HPP_
#define EXPERIMENTAL__MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <tier4_v2x_msgs/InfrastructureCommandArray.h>
#include <tier4_v2x_msgs/VirtualTrafficLightStateArray.h>

#include <functional>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
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

  void plan(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound,
    const PlannerData & planner_data) override;

private:
  VirtualTrafficLightModule::PlannerParam planner_param_;

  void launchNewModules(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<VirtualTrafficLightModule> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;

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

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
