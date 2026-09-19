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

#include "scene_no_stopping_area.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <autoware/behavior_velocity_no_stopping_area_module/ros1_support.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class NoStoppingAreaModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit NoStoppingAreaModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "no_stopping_area"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    // The scene dereferences predicted_objects when checking blocked entry.
    // Require the host to receive it before executing this module.
    RequiredSubscriptionInfo required;
    required.predicted_objects = true;
    return required;
  }

private:
  std::string logger_;
  std::shared_ptr<NoStoppingAreaRos1Clock> clock_{std::make_shared<NoStoppingAreaRos1Clock>()};
  NoStoppingAreaModule::PlannerParam planner_param_{};

  void launchNewModules(const autoware_internal_planning_msgs::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(
    const autoware_internal_planning_msgs::PathWithLaneId & path) override;
};

class NoStoppingAreaModulePlugin : public PluginWrapper<NoStoppingAreaModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
