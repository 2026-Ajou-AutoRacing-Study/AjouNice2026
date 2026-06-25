> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_planner

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner`
- Version: `1.8.0`
- Package description: Node of the motion_velocity_planner
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::motion_velocity_planner::MotionVelocityPlannerNode` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |

## Purpose And Role

`motion_velocity_planner` is a planner to adjust the trajectory velocity based on the obstacles around the vehicle. It loads modules as plugins. Please refer to the links listed below for detail on each module. Each module calculates stop and slow down points to be inserted in the ego trajectory. These points are assumed to correspond to the `base_link` frame of the ego vehicle as it follows the trajectory. This means that to stop before a wall, a stop point is inserted in the trajectory at a distance ahead of the wall equal to the vehicle front offset (wheelbase + front overhang, see the [vehicle dimensions](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-ar

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/dynamic_objects` | `autoware_perception_msgs::msg::PredictedObjects` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.hpp` |
| `~/input/no_ground_pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.hpp` |
| `~/input/vehicle_odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.hpp` |
| `~/input/accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.hpp` |
| `~/input/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.hpp` |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.hpp` |
| `~/input/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |
| `~/output/velocity_limit` | `VelocityLimit` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |
| `~/output/clear_velocity_limit` | `VelocityLimitClearCommand` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |
| `~/debug/markers` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |
| `~/debug/processed_pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `~/service/load_plugin` | `LoadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |
| `~/service/unload_plugin` | `UnloadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `smooth_velocity_before_planning` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | publisher | `/motion_velocity_planner/input/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/dynamic_objects` | `autoware_perception_msgs::msg::PredictedObjects` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/no_ground_pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/vehicle_odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/motion_velocity_planner/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | publisher | `/tf` | `tf2_msgs::msg::TFMessage` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | subscription | `/motion_velocity_planner/output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | subscription | `/motion_velocity_planner/output/velocity_limit` | `autoware_internal_planning_msgs::msg::VelocityLimit` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | subscription | `/motion_velocity_planner/output/clear_velocity_limit` | `autoware_internal_planning_msgs::msg::VelocityLimitClearCommand` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | service_or_client | `/motion_velocity_planner/service/load_plugin` | `autoware_internal_planning_msgs::srv::LoadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | service_or_client | `/motion_velocity_planner/service/unload_plugin` | `autoware_internal_planning_msgs::srv::UnloadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | parameter | `smooth_velocity_before_planning` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |
| test | parameter | `smooth_velocity_before_planning` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test/test_node.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `smooth_velocity_before_planning` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.decimate_trajectory_step_length` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.goal_extended_trajectory_length` | `6.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.consider_current_pose.time_to_convergence` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.enable_monolithic_crop_box` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.enable_multi_polygon_filtering` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.braking_distance_scale_factor` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.lateral_margin` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.height_margin` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.enable_downsample` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_x` | `0.05` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_y` | `0.05` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_z` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.enable_clustering` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.cluster_tolerance` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.min_cluster_size` | `1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.max_cluster_size` | `100000` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml` |
| `wheel_radius` | `0.39` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `wheel_width` | `0.42` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `wheel_base` | `2.74` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `wheel_tread` | `1.63` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `front_overhang` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `rear_overhang` | `1.03` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `left_overhang` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `right_overhang` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `vehicle_height` | `2.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `max_steer_angle` | `0.70` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/vehicle_info.param.yaml` |
| `max_vel` | `4.17` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `normal.min_acc` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `normal.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `normal.min_jerk` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `normal.max_jerk` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `limit.min_acc` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `limit.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `limit.min_jerk` | `-1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `limit.max_jerk` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/common.param.yaml` |
| `ego_nearest_dist_threshold` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/nearest_search.param.yaml` |
| `ego_nearest_yaw_threshold` | `1.046` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/nearest_search.param.yaml` |
| `smooth_velocity_before_planning` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.decimate_trajectory_step_length` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.goal_extended_trajectory_length` | `6.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.consider_current_pose.time_to_convergence` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.enable_monolithic_crop_box` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.enable_multi_polygon_filtering` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.braking_distance_scale_factor` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.lateral_margin` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.height_margin` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.enable_downsample` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_x` | `0.05` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_y` | `0.05` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_z` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.enable_clustering` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.cluster_tolerance` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.min_cluster_size` | `1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.max_cluster_size` | `100000` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `obstacle_stop.option.ignore_crossing_obstacle` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.option.suppress_sudden_stop` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_margin` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.terminal_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_behavior_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.behavior_stop_margin_hold_time` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.max_negative_velocity` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_margin_opposing_traffic` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.effective_deceleration_opposing_traffic` | `4.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_on_duration` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_off_duration` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.update_distance_th` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.hold_stop_velocity_threshold` | `0.01` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.hold_stop_distance_threshold` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.pointcloud_suppression_distance_margin` | `0.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.enable_approaching` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.additional_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.min_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.limit_min_acc` | `-1.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.sudden_object_acc_threshold` | `-1.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.sudden_object_dist_threshold` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.abandon_to_stop` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.use_rss_stop` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.two_wheel_objects_deceleration` | `-3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.vehicle_objects_deceleration` | `-1.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.no_wheel_objects_deceleration` | `-10.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.pointcloud_deceleration` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.velocity_offset` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_enter_fixed_stop` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_exit_fixed_stop` | `3.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_inside.default` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_inside.pointcloud` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_outside.default` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_outside.unknown` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.default` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.pointcloud` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.braking_distance_scale_factor` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.default` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.unknown` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.pointcloud` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.wheel_off_track_scale.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_threshold_velocity` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_stop_obstacle.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_obstacle.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.default` | `50.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.unknown` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.bottom_limit` | `-50.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.min_object_length` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.min_velocity_to_reach_collision_point` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.stop_obstacle_hold_time_threshold` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.estimation_time_horizon` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_lateral_velocity` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.min_longitudinal_velocity` | `-8.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_moving_direction_angle` | `2.44` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.pedestrian` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.bicycle` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.collision_time_margin` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.traj_angle_threshold` | `0.523599` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_time_diff` | `0.35` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.min_velocity` | `-5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_velocity` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.position_diff` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.use_estimated_velocity` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.min_clamp_velocity` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.max_clamp_velocity` | `20.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.required_velocity_count` | `5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.lpf_gain` | `0.9` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_top` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_bottom` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `resample.ds_resample` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `resample.num_resample` | `1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `resample.delta_yaw_threshold` | `0.785` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `latacc.enable_constant_velocity_while_turning` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `latacc.constant_velocity_dist_threshold` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.min_acc` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.max_jerk` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.min_jerk` | `-0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.kp` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.start_jerk` | `-0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_jerk_mild_stop` | `-0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_jerk` | `-1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_acc_mild_stop` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_acc` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.span_jerk` | `-0.01` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/Analytical.param.yaml` |
| `max_velocity` | `20.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stop_decel` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `margin_to_insert_external_velocity_limit` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `curvature_calculation_distance` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `enable_lateral_acc_limit` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `lateral_acceleration_limits` | `[0.8, 0.8, 0.8, 0.8]` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `min_curve_velocity` | `2.74` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `decel_distance_before_curve` | `3.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `decel_distance_after_curve` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `min_decel_for_lateral_acc_lim_filter` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `enable_steering_rate_limit` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `velocity_thresholds` | `[0.1, 0.3, 20.0, 30.0]` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `steering_angle_rate_limits` | `[57.0, 57.0, 57.0, 40.0]` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `resample_ds` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `curvature_threshold` | `0.02` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `replan_vel_deviation` | `5.53` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `engage_velocity` | `0.25` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `engage_acceleration` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `engage_exit_ratio` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stop_dist_to_prohibit_engage` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stopping_velocity` | `2.778` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stopping_distance` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `extract_ahead_dist` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `extract_behind_dist` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `delta_yaw_threshold` | `1.0472` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `max_trajectory_length` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `min_trajectory_length` | `150.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `resample_time` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `dense_resample_dt` | `0.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `dense_min_interval_distance` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `sparse_resample_dt` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `sparse_min_interval_distance` | `4.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_max_trajectory_length` | `300.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_resample_time` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_dense_resample_dt` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_dense_min_interval_distance` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_sparse_resample_dt` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_sparse_min_interval_distance` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `over_stop_velocity_warn_thr` | `1.389` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `plan_from_ego_speed_on_manual_mode` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/test_config/autoware_velocity_smoother/velocity_smoother.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_test_utils`, `autoware_utils_debug`, `autoware_utils_geometry`, `autoware_utils_logging`, `autoware_utils_pcl`, `autoware_utils_rclcpp`, `autoware_utils_system`, `autoware_velocity_smoother`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `grid_map_core`, `libboost-dev`, `pcl_conversions`, `pluginlib`, `rclcpp`, `rclcpp_components`, `rosidl_default_generators`, `rosidl_default_runtime`, `sensor_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 75 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 12 |
| `pluginlib` | 3 |
| `parameters` | 4 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/README.md`
