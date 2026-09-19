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

#include "autoware/behavior_velocity_blind_spot_module/parameter.hpp"
#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner::experimental
{

class BlindSpotModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit BlindSpotModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "blind_spot"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    return required_subscription_info;
  }

private:
  std::string logger_;
  std::shared_ptr<BlindSpotRos1Clock> clock_{std::make_shared<BlindSpotRos1Clock>()};
  BlindSpotPlannerParam planner_param_;

  void launchNewModules(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const Trajectory & path, const PlannerData & planner_data) override;

  ros::Publisher decision_state_pub_;
};

class BlindSpotModulePlugin : public PluginWrapper<BlindSpotModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
