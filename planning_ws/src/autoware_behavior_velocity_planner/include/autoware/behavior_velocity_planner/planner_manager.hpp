// Copyright 2019 Autoware Foundation
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__PLANNER_MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__PLANNER_MANAGER_HPP_

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

class BehaviorVelocityPlannerManager
{
public:
  BehaviorVelocityPlannerManager();
  void launchScenePlugin(ros::NodeHandle & node, const std::string & name);
  void removeScenePlugin(ros::NodeHandle & node, const std::string & name);

  // cppcheck-suppress functionConst
  autoware_internal_planning_msgs::PathWithLaneId planPathVelocity(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_internal_planning_msgs::PathWithLaneId & input_path_msg);

  RequiredSubscriptionInfo getRequiredSubscriptions() const { return required_subscriptions_; }

bool isPluginAvailable(const std::string & name) {
    return common_plugin_loader_.isClassAvailable(name) || plugin_loader_.isClassAvailable(name);
  }
  bool hasModule(const std::string & name) const {
    for (const auto & plugin : scene_manager_plugins_) if (plugin->getModuleName()==name) return true;
    return false;
  }
private:
  pluginlib::ClassLoader<PluginInterface> plugin_loader_;
  pluginlib::ClassLoader<PluginInterface> common_plugin_loader_;
  std::vector<std::shared_ptr<PluginInterface>> scene_manager_plugins_;
  RequiredSubscriptionInfo required_subscriptions_;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__PLANNER_MANAGER_HPP_
