> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_trajectory_modifier

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_trajectory_modifier`
- Version: `0.51.0`
- Package description: The autoware_trajectory_modifier package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::trajectory_modifier::TrajectoryModifier` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier.cpp` |
| pluginlib_plugin | `autoware::trajectory_modifier::plugin::ObstacleStop -> autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier_plugins/obstacle_stop.cpp` |
| pluginlib_plugin | `autoware::trajectory_modifier::plugin::StopPointFixer -> autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier_plugins/stop_point_fixer.cpp` |

## Purpose And Role

The `autoware_trajectory_modifier` package provides a plugin-based architecture for post-processing trajectory points to improve trajectory quality and ensure vehicle safety. It takes candidate trajectories and applies various modification algorithms to enhance their feasibility and safety characteristics. - Plugin-based architecture for extensible trajectory modifications - Stop point fixing to prevent trajectory issues near stationary conditions

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/candidate_trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier.hpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier.hpp` |
| `~/input/pointcloud` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/candidate_trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier.cpp` |
| `~/debug/processing_time_detail` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier.cpp` |
| `~/obstacle_stop/debug/cluster_points` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier_plugins/obstacle_stop.cpp` |
| `~/obstacle_stop/debug/marker` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/src/trajectory_modifier_plugins/obstacle_stop.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| _none detected by literal scan_ |  |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `use_stop_point_fixer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `use_obstacle_stop` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `trajectory_time_step` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `stop_point_fixer.force_stop_long_stopped_trajectories` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `stop_point_fixer.force_stop_close_stopped_trajectories` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `stop_point_fixer.velocity_threshold` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `stop_point_fixer.min_distance_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `stop_point_fixer.min_stop_duration` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.use_objects` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.use_pointcloud` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.enable_stop_for_objects` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.enable_stop_for_pointcloud` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.stop_margin` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.nominal_stopping_decel` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.maximum_stopping_decel` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.stopping_jerk` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.lateral_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.arrived_distance_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.obstacle_tracking.on_time_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.obstacle_tracking.off_time_buffer` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.obstacle_tracking.object_distance_th` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.obstacle_tracking.object_yaw_th` | `0.1745` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.obstacle_tracking.pcd_distance_th` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.obstacle_tracking.grace_period` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.objects.object_types` | `["car", "truck", "bus", "trailer", "motorcycle", "bicycle", "pedestrian"]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.objects.max_velocity_th` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.height_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.min_height` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.voxel_grid_filter.x` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.voxel_grid_filter.y` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.voxel_grid_filter.z` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.voxel_grid_filter.min_size` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.clustering.tolerance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.clustering.min_height` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.clustering.min_size` | `10` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.pointcloud.clustering.max_size` | `10000` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.car` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.truck` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.bus` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.trailer` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.motorcycle` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.bicycle` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.object_decel.pedestrian` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.reaction_time` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.safety_margin` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |
| `obstacle_stop.rss_params.min_vel_th` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_path_optimizer`, `autoware_path_smoother`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_signal_processing`, `autoware_trajectory`, `autoware_utils`, `autoware_utils_debug`, `autoware_utils_rclcpp`, `autoware_vehicle_info_utils`, `generate_parameter_library`, `geometry_msgs`, `nav_msgs`, `pluginlib`, `range-v3`, `rclcpp`, `rclcpp_components`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 50 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 12 |
| `pluginlib` | 7 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_modifier/README.md`
