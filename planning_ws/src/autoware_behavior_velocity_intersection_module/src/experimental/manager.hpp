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

#include "scene_intersection.hpp"
#include "scene_merge_from_private_road.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <functional>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class IntersectionModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit IntersectionModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "intersection"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.traffic_signals = true;
    required_subscription_info.predicted_objects = true;
    required_subscription_info.occupancy_grid_map = intersection_param_.occlusion.enable;
    return required_subscription_info;
  }

private:
  std::string logger_;
  std::shared_ptr<IntersectionRos1Clock> clock_{std::make_shared<IntersectionRos1Clock>()};
  IntersectionModule::PlannerParam intersection_param_;
  // additional for INTERSECTION_OCCLUSION
  RTCInterface occlusion_rtc_interface_;

  void launchNewModules(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const Trajectory & path, const PlannerData & planner_data) override;

  bool hasSameParentLaneletAndTurnDirectionWithRegistered(const lanelet::ConstLanelet & lane) const;

  /* called from SceneModuleInterfaceWithRTC::plan */
  void sendRTC(const Time & stamp) override;
  void setActivation() override;
  void modifyPathVelocity(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound,
    const PlannerData & planner_data) override;
  /* called from SceneModuleInterface::updateSceneModuleInstances */
  void deleteExpiredModules(const Trajectory & path, const PlannerData & planner_data) override;

  ros::Publisher decision_state_pub_;
  ros::Publisher
    tl_observation_pub_;

  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_for_occlusion_;
};

class MergeFromPrivateModuleManager : public experimental::SceneModuleManagerInterface<>
{
public:
  explicit MergeFromPrivateModuleManager(ros::NodeHandle & node);

  const char * getModuleName() override { return "merge_from_private"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  std::string logger_;
  std::shared_ptr<IntersectionRos1Clock> clock_{std::make_shared<IntersectionRos1Clock>()};
  MergeFromPrivateRoadModule::PlannerParam merge_from_private_area_param_;

  void launchNewModules(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;

  bool hasSameParentLaneletAndTurnDirectionWithRegistered(const lanelet::ConstLanelet & lane) const;
};

class IntersectionModulePlugin : public PluginWrapper<IntersectionModuleManager>
{
};

class MergeFromPrivateModulePlugin : public PluginWrapper<MergeFromPrivateModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
