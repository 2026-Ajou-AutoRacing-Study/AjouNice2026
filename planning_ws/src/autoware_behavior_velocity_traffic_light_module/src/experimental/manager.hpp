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

#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <functional>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class TrafficLightModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit TrafficLightModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "traffic_light"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.traffic_signals = true;
    return required_subscription_info;
  }

private:
  std::string logger_;
  std::shared_ptr<TrafficLightRos1Clock> clock_{std::make_shared<TrafficLightRos1Clock>()};
  TrafficLightModule::PlannerParam planner_param_;

  void launchNewModules(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const Trajectory & path, const PlannerData & planner_data) override;

  void modifyPathVelocity(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound,
    const PlannerData & planner_data) override;

  bool isModuleRegisteredFromRegElement(const lanelet::Id & id, const size_t module_id) const;

  std::shared_ptr<TrafficLightModule> getRegisteredAssociatedModule(
    const lanelet::Id & id, const PlannerData & planner_data) const;

  bool hasAssociatedTrafficLight(
    const lanelet::ConstLanelet & lane, const lanelet::Id & registered_id,
    const PlannerData & planner_data) const;

  std::shared_ptr<TrafficLightModule> findModuleById(const lanelet::Id & module_id) const;

  bool hasSameTrafficLight(
    const lanelet::TrafficLightConstPtr element,
    const lanelet::TrafficLightConstPtr registered_element) const;

  bool hasStaticArrow(
    const std::shared_ptr<const lanelet::autoware::AutowareTrafficLight> & reg_elem) const;

  // Debug
  ros::Publisher pub_tl_state_;
};

class TrafficLightModulePlugin : public PluginWrapper<TrafficLightModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
