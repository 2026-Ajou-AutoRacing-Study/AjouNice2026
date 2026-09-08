// Copyright 2025 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
namespace autoware::motion_velocity_planner {
namespace {
template<class T> T get_or_declare_parameter(const ros::NodeHandle & node,const std::string & name) {
  return autoware_utils_rclcpp::detail::strict_get<T>(node,autoware_utils_rclcpp::to_ros1_parameter_name(name));
}
}
PlannerData::PointcloudPreprocessParams::PointcloudPreprocessParams(ros::NodeHandle & node)
  {
    std::string ns = "pointcloud_preprocessing.";
    {
      std::string ns_child = ns + "filter_by_trajectory_polygon.";
      filter_by_trajectory_polygon.enable_monolithic_crop_box =
        get_or_declare_parameter<bool>(node, ns_child + "enable_monolithic_crop_box");
      filter_by_trajectory_polygon.enable_multi_polygon_filtering =
        get_or_declare_parameter<bool>(node, ns_child + "enable_multi_polygon_filtering");
      filter_by_trajectory_polygon.min_trajectory_length =
        get_or_declare_parameter<double>(node, ns_child + "min_trajectory_length");
      filter_by_trajectory_polygon.braking_distance_scale_factor =
        get_or_declare_parameter<double>(node, ns_child + "braking_distance_scale_factor");
      filter_by_trajectory_polygon.lateral_margin =
        get_or_declare_parameter<double>(node, ns_child + "lateral_margin");
      filter_by_trajectory_polygon.height_margin =
        get_or_declare_parameter<double>(node, ns_child + "height_margin");
    }
    {
      std::string ns_child = ns + "downsample_by_voxel_grid.";
      downsample_by_voxel_grid.enable_downsample =
        get_or_declare_parameter<bool>(node, ns_child + "enable_downsample");
      downsample_by_voxel_grid.voxel_size_x =
        get_or_declare_parameter<double>(node, ns_child + "voxel_size_x");
      downsample_by_voxel_grid.voxel_size_y =
        get_or_declare_parameter<double>(node, ns_child + "voxel_size_y");
      downsample_by_voxel_grid.voxel_size_z =
        get_or_declare_parameter<double>(node, ns_child + "voxel_size_z");
    }
    {
      std::string ns_child = ns + "euclidean_clustering.";
      euclidean_clustering.enable_clustering =
        get_or_declare_parameter<bool>(node, ns_child + "enable_clustering");
      euclidean_clustering.cluster_tolerance =
        get_or_declare_parameter<double>(node, ns_child + "cluster_tolerance");
      euclidean_clustering.min_cluster_size =
        get_or_declare_parameter<int>(node, ns_child + "min_cluster_size");
      euclidean_clustering.max_cluster_size =
        get_or_declare_parameter<int>(node, ns_child + "max_cluster_size");
    }
  }
PlannerData::PlannerData(ros::NodeHandle & node)
: no_ground_pointcloud(node),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())

{
  // nearest search
  ego_nearest_dist_threshold = get_or_declare_parameter<double>(node, "ego_nearest_dist_threshold");
  ego_nearest_yaw_threshold = get_or_declare_parameter<double>(node, "ego_nearest_yaw_threshold");

  trajectory_polygon_collision_check.decimate_trajectory_step_length =
    get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.decimate_trajectory_step_length");
  trajectory_polygon_collision_check.goal_extended_trajectory_length =
    get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.goal_extended_trajectory_length");
  trajectory_polygon_collision_check.enable_to_consider_current_pose =
    get_or_declare_parameter<bool>(
      node,
      "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose");
  trajectory_polygon_collision_check.time_to_convergence = get_or_declare_parameter<double>(
    node, "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence");
}
}
