#include "full_native_support.hpp"
// Copyright 2024 Autoware Foundation
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

#ifndef PLANNER_MANAGER_HPP_
#define PLANNER_MANAGER_HPP_

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <pluginlib/class_loader.hpp>
#include "full_native_support.hpp"
#include <tf2_ros/transform_listener.h>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <sensor_msgs/PointCloud2.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
class FullSourceMotionVelocityPlannerManager
{
public:
  FullSourceMotionVelocityPlannerManager();
  void load_module_plugin(ros::NodeHandle & node, const std::string & name);
  void unload_module_plugin(ros::NodeHandle & node, const std::string & name);
  void update_module_parameters(const ParameterMap & parameters);
  std::vector<VelocityPlanningResult> plan_velocities(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data,
    std::shared_ptr<autoware_utils_debug::DebugPublisher> & processing_time_publisher);

  size_t size() const { return loaded_plugins_.size(); }
  bool has_module(const std::string & name) const {
    return std::any_of(loaded_plugins_.begin(),loaded_plugins_.end(),
      [&](const auto & plugin){return plugin->getModuleName()==name;});
  }
  RequiredSubscriptionInfo getRequiredSubscriptions() const { return required_subscriptions_; }

private:
  pluginlib::ClassLoader<PluginModuleInterface> plugin_loader_;
  std::vector<std::shared_ptr<PluginModuleInterface>> loaded_plugins_;
  RequiredSubscriptionInfo required_subscriptions_;
  std::map<std::string,ros::Publisher> debug_trajectory_publishers_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // PLANNER_MANAGER_HPP_
