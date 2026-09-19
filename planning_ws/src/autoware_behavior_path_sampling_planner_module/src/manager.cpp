#include <autoware_utils_rclcpp/parameter_event.hpp>
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

#include "autoware/behavior_path_sampling_planner_module/manager.hpp"

#include <ros/ros.h>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

namespace
{
template <class T>
void requireParam(ros::NodeHandle * node, const std::string & key, T * value)
{
  if (!node || !node->getParam(key, *value)) throw std::runtime_error("missing sampling-planner parameter: " + key);
}
}

void SamplingPlannerModuleManager::init(ros::NodeHandle * node)
{
  if (!SceneModuleManagerInterface::init(node, {""})) {
    throw std::runtime_error("sampling-planner provider initialization failed");
  }

  reloadParameters();
}

bool SamplingPlannerModuleManager::initialize(ros::NodeHandle * node)
{
  init(node);
  return true;
}

void SamplingPlannerModuleManager::reloadParameters()
{
  if (!node_) throw std::runtime_error("sampling-planner manager has no ROS1 node handle");

  SamplingPlannerParameters p{};
  {
    // ROS2 dotted parameter namespaces are not valid ROS1 graph resource
    // names.  The rendered YAML already uses nested maps, so consume the
    // corresponding ROS1 slash paths rather than relying on an invalid key.
    std::string ns{"constraints/hard"};
    requireParam(node_, ns + "/max_curvature", &p.max_curvature);
    requireParam(node_, ns + "/min_curvature", &p.min_curvature);
    ns = std::string{"constraints/soft"};
    requireParam(node_, ns + "/lateral_deviation_weight", &p.lateral_deviation_weight);
    requireParam(node_, ns + "/length_weight", &p.length_weight);
    requireParam(node_, ns + "/curvature_weight", &p.curvature_weight);
    requireParam(node_, ns + "/weights", &p.weights);
  }
  {
    std::string ns{"sampling"};
    requireParam(node_, ns + "/enable_frenet", &p.enable_frenet);
    requireParam(node_, ns + "/enable_bezier", &p.enable_bezier);
    requireParam(node_, ns + "/resolution", &p.resolution);
    requireParam(node_, ns + "/previous_path_reuse_points_nb", &p.previous_path_reuse_points_nb);
    requireParam(node_, ns + "/nb_target_lateral_positions", &p.nb_target_lateral_positions);
    requireParam(node_, ns + "/target_lengths", &p.target_lengths);
    requireParam(node_, ns + "/target_lateral_positions", &p.target_lateral_positions);
    ns += "/frenet";
    requireParam(node_, ns + "/target_lateral_velocities", &p.target_lateral_velocities);
    requireParam(node_, ns + "/target_lateral_accelerations", &p.target_lateral_accelerations);
  }
  {
    std::string ns{"preprocessing"};
    requireParam(node_, ns + "/force_zero_initial_deviation", &p.force_zero_deviation);
    requireParam(node_, ns + "/force_zero_initial_heading", &p.force_zero_heading);
    requireParam(node_, ns + "/smooth_reference_trajectory", &p.smooth_reference);
  }
  parameters_ = std::make_shared<SamplingPlannerParameters>(p);

  for (const auto & module : activeModules()) {
    if (const auto sampling_planner_ptr = std::dynamic_pointer_cast<SamplingPlannerModule>(module)) {
      sampling_planner_ptr->updateModuleParams(parameters_);
    }
  }
}

void SamplingPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto & p = parameters_;

  {
    std::string ns{"constraints.hard"};
    update_param<double>(parameters, ns + ".max_curvature", p->max_curvature);
    update_param<double>(parameters, ns + ".min_curvature", p->min_curvature);
    ns = std::string{"constraints.soft"};
    update_param<double>(parameters, ns + ".lateral_deviation_weight", p->lateral_deviation_weight);
    update_param<double>(parameters, ns + ".length_weight", p->length_weight);
    update_param<double>(parameters, ns + ".curvature_weight", p->curvature_weight);
    update_param<std::vector<double>>(parameters, ns + ".weights", p->weights);
  }
  {
    std::string ns{"sampling"};
    update_param<bool>(parameters, ns + ".enable_frenet", p->enable_frenet);
    update_param<bool>(parameters, ns + ".enable_bezier", p->enable_bezier);
    update_param<double>(parameters, ns + ".resolution", p->resolution);

    update_param<int>(
      parameters, ns + ".previous_path_reuse_points_nb", p->previous_path_reuse_points_nb);

    update_param<int>(
      parameters, ns + ".nb_target_lateral_positions", p->nb_target_lateral_positions);
    update_param<std::vector<double>>(parameters, ns + ".target_lengths", p->target_lengths);

    update_param<std::vector<double>>(
      parameters, ns + ".target_lateral_positions", p->target_lateral_positions);

    ns += ".frenet";
    update_param<std::vector<double>>(
      parameters, ns + ".target_lateral_velocities", p->target_lateral_velocities);

    update_param<std::vector<double>>(
      parameters, ns + ".target_lateral_accelerations", p->target_lateral_accelerations);
  }
  {
    std::string ns{"preprocessing"};
    update_param<bool>(parameters, ns + ".force_zero_initial_deviation", p->force_zero_deviation);
    update_param<bool>(parameters, ns + ".force_zero_initial_heading", p->force_zero_heading);
    update_param<bool>(parameters, ns + ".smooth_reference_trajectory", p->smooth_reference);
  }

  for (const auto & module : activeModules()) {
    if (const auto sampling_planner = std::dynamic_pointer_cast<SamplingPlannerModule>(module)) {
      sampling_planner->updateModuleParams(p);
    }
  }
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SamplingPlannerModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
