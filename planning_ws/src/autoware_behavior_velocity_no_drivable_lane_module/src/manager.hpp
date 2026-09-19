// Copyright 2025 TIER IV, Inc.
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

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <ros/ros.h>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class NoDrivableLaneModuleManager : public experimental::SceneModuleManagerInterface<>
{
public:
  explicit NoDrivableLaneModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "no_drivable_lane"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  NoDrivableLaneModule::PlannerParam planner_param_;

  void launchNewModules(
    const experimental::Trajectory & path, const ros::Time & stamp,
    const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
  getModuleExpiredFunction(
    const experimental::Trajectory & path, const PlannerData & planner_data) override;
};

class NoDrivableLaneModulePlugin : public experimental::PluginWrapper<NoDrivableLaneModuleManager>
{
};

namespace experimental
{
class NoDrivableLaneModulePlugin
: public ::autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin
{
};
}  // namespace experimental

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
