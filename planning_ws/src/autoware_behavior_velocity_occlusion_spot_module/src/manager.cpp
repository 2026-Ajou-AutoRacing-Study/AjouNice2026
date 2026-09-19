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

#include "manager.hpp"

#include "scene_occlusion_spot.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using occlusion_spot_ros1::require_parameter;
using occlusion_spot_utils::DETECTION_METHOD;
using occlusion_spot_utils::PASS_JUDGE;

OcclusionSpotModuleManager::OcclusionSpotModuleManager(ros::NodeHandle & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(OcclusionSpotModuleManager::getModuleName());
  auto & pp = planner_param_;
  // for detection type
  {
    const std::string method =
      require_parameter<std::string>(node, ns + ".detection_method");
    if (method == "occupancy_grid") {  // module id 0
      pp.detection_method = DETECTION_METHOD::OCCUPANCY_GRID;
      module_id_ = DETECTION_METHOD::OCCUPANCY_GRID;
    } else if (method == "predicted_object") {  // module id 1
      pp.detection_method = DETECTION_METHOD::PREDICTED_OBJECT;
      module_id_ = DETECTION_METHOD::PREDICTED_OBJECT;
    } else {
      throw std::invalid_argument{
        "[behavior_velocity]: occlusion spot detection method has invalid argument"};
    }
  }
  // for passable judgement
  {
    const std::string pass_judge = require_parameter<std::string>(node, ns + ".pass_judge");
    if (pass_judge == "current_velocity") {
      pp.pass_judge = PASS_JUDGE::CURRENT_VELOCITY;
    } else if (pass_judge == "smooth_velocity") {
      pp.pass_judge = PASS_JUDGE::SMOOTH_VELOCITY;
    } else {
      throw std::invalid_argument{
        "[behavior_velocity]: occlusion spot pass judge method has invalid argument"};
    }
  }
  pp.use_object_info = require_parameter<bool>(node, ns + ".use_object_info");
  pp.use_moving_object_ray_cast =
    require_parameter<bool>(node, ns + ".use_moving_object_ray_cast");
  pp.use_partition_lanelet = require_parameter<bool>(node, ns + ".use_partition_lanelet");
  pp.pedestrian_vel = require_parameter<double>(node, ns + ".pedestrian_vel");
  pp.pedestrian_radius = require_parameter<double>(node, ns + ".pedestrian_radius");

  // debug
  pp.is_show_occlusion = require_parameter<bool>(node, ns + ".debug.is_show_occlusion");
  pp.is_show_cv_window = require_parameter<bool>(node, ns + ".debug.is_show_cv_window");
  pp.is_show_processing_time =
    require_parameter<bool>(node, ns + ".debug.is_show_processing_time");

  // threshold
  pp.detection_area_offset =
    require_parameter<double>(node, ns + ".threshold.detection_area_offset");
  pp.detection_area_length =
    require_parameter<double>(node, ns + ".threshold.detection_area_length");
  pp.stuck_vehicle_vel =
    require_parameter<double>(node, ns + ".threshold.stuck_vehicle_vel");
  pp.lateral_distance_thr =
    require_parameter<double>(node, ns + ".threshold.lateral_distance");
  pp.dist_thr = require_parameter<double>(node, ns + ".threshold.search_dist");
  pp.angle_thr = require_parameter<double>(node, ns + ".threshold.search_angle");

  // ego additional velocity config
  pp.v.safety_ratio = require_parameter<double>(node, ns + ".motion.safety_ratio");
  pp.v.safe_margin = require_parameter<double>(node, ns + ".motion.safe_margin");
  pp.v.max_slow_down_jerk =
    require_parameter<double>(node, ns + ".motion.max_slow_down_jerk");
  pp.v.max_slow_down_accel =
    require_parameter<double>(node, ns + ".motion.max_slow_down_accel");
  pp.v.non_effective_jerk =
    require_parameter<double>(node, ns + ".motion.non_effective_jerk");
  pp.v.non_effective_accel =
    require_parameter<double>(node, ns + ".motion.non_effective_acceleration");
  pp.v.min_allowed_velocity =
    require_parameter<double>(node, ns + ".motion.min_allowed_velocity");
  // detection_area param
  pp.detection_area.min_occlusion_spot_size =
    require_parameter<double>(node, ns + ".detection_area.min_occlusion_spot_size");
  pp.detection_area.min_longitudinal_offset =
    require_parameter<double>(node, ns + ".detection_area.min_longitudinal_offset");
  pp.detection_area.max_lateral_distance =
    require_parameter<double>(node, ns + ".detection_area.max_lateral_distance");
  pp.detection_area.slice_length =
    require_parameter<double>(node, ns + ".detection_area.slice_length");
  // occupancy grid param
  pp.grid.free_space_max = require_parameter<int>(node, ns + ".grid.free_space_max");
  pp.grid.occupied_min = require_parameter<int>(node, ns + ".grid.occupied_min");

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  pp.baselink_to_front = vehicle_info.max_longitudinal_offset_m;
  pp.wheel_tread = vehicle_info.wheel_tread_m;
  pp.right_overhang = vehicle_info.right_overhang_m;
  pp.left_overhang = vehicle_info.left_overhang_m;
}

void OcclusionSpotModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  if (path.points.empty()) return;
  // general
  if (!isModuleRegistered(module_id_)) {
    registerModule(
      std::make_shared<OcclusionSpotModule>(
        module_id_, planner_data_, planner_param_, *node_, (logger_ + ".occlusion_spot_module"),
        clock_, time_keeper_, planning_factor_interface_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
OcclusionSpotModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  return [path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return false;
  };
}
}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::OcclusionSpotModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
