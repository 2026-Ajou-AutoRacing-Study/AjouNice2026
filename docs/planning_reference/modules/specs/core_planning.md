> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_core_planning

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/autoware_core_planning`
- Version: `1.8.0`
- Package description: The autoware_core_planning package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| _none detected_ |  |  |

## Purpose And Role

The `autoware_core_planning` is a meta-package within the Autoware project, designed to manage and organize various planning-related modules essential for autonomous vehicle operations. This package serves as a central point for integrating different planning functionalities, such as path planning, path optimization, and trajectory generation. The `autoware_core_planning` meta-package is used to launch and manage planning modules as part of the Autoware system. You can start the system using:

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

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
| `max_vel` | `4.17` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `normal.min_acc` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `normal.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `normal.min_jerk` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `normal.max_jerk` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `limit.min_acc` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `limit.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `limit.min_jerk` | `-1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `limit.max_jerk` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml` |
| `planning_hz` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `path_length.backward` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `path_length.forward` | `300.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `waypoint_group.separation_threshold` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `waypoint_group.interval_margin_ratio` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `turn_signal.search_time` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `turn_signal.search_distance` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `turn_signal.resampling_interval` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `turn_signal.angle_threshold_deg` | `15.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `refine_goal_search_radius_range` | `7.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `search_radius_decrement` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/path_generator.param.yaml` |
| `ego_nearest_dist_threshold` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/nearest_search.param.yaml` |
| `ego_nearest_yaw_threshold` | `1.046` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/nearest_search.param.yaml` |
| `max_accel` | `-2.8` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml` |
| `max_jerk` | `-5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml` |
| `system_delay` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml` |
| `delay_response_time` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml` |
| `is_publish_debug_path` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml` |
| `stop_line.stop_margin` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/stop_line.param.yaml` |
| `stop_line.required_stop_duration_sec` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/stop_line.param.yaml` |
| `stop_line.vehicle_stopped_duration_threshold` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/stop_line.param.yaml` |
| `stop_line.hold_stop_margin_distance` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/stop_line.param.yaml` |
| `forward_path_length` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner.param.yaml` |
| `backward_path_length` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner.param.yaml` |
| `behavior_output_path_interval` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner.param.yaml` |
| `planning_factor_console_output.enable` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner.param.yaml` |
| `planning_factor_console_output.duration` | `1000` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/behavior_velocity_planner/behavior_velocity_planner.param.yaml` |
| `smooth_velocity_before_planning` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.decimate_trajectory_step_length` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.goal_extended_trajectory_length` | `6.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `trajectory_polygon_collision_check.consider_current_pose.time_to_convergence` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.enable_monolithic_crop_box` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.enable_multi_polygon_filtering` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.braking_distance_scale_factor` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.lateral_margin` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.filter_by_trajectory_polygon.height_margin` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.enable_downsample` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_x` | `0.05` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_y` | `0.05` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_z` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.enable_clustering` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.cluster_tolerance` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.min_cluster_size` | `1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `pointcloud_preprocessing.euclidean_clustering.max_cluster_size` | `100000` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/motion_velocity_planner.param.yaml` |
| `obstacle_stop.option.ignore_crossing_obstacle` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.option.suppress_sudden_stop` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_margin` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.terminal_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_behavior_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.behavior_stop_margin_hold_time` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.max_negative_velocity` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_margin_opposing_traffic` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.effective_deceleration_opposing_traffic` | `4.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_on_duration` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_off_duration` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.update_distance_th` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.hold_stop_velocity_threshold` | `0.01` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.hold_stop_distance_threshold` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.pointcloud_suppression_distance_margin` | `0.2` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.enable_approaching` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.additional_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.min_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.limit_min_acc` | `-1.2` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.sudden_object_acc_threshold` | `-1.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.sudden_object_dist_threshold` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.abandon_to_stop` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.use_rss_stop` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.two_wheel_objects_deceleration` | `-3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.vehicle_objects_deceleration` | `-1.2` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.no_wheel_objects_deceleration` | `-10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.pointcloud_deceleration` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.velocity_offset` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_enter_fixed_stop` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_exit_fixed_stop` | `3.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_inside.default` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_inside.pointcloud` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_outside.default` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_outside.unknown` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.default` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.pointcloud` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.braking_distance_scale_factor` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.default` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.unknown` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.pointcloud` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.wheel_off_track_scale.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_threshold_velocity` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_stop_obstacle.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_obstacle.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.default` | `50.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.unknown` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.bottom_limit` | `-50.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.min_object_length` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.min_velocity_to_reach_collision_point` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.stop_obstacle_hold_time_threshold` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.estimation_time_horizon` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_lateral_velocity` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.min_longitudinal_velocity` | `-8.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_moving_direction_angle` | `2.44` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.pedestrian` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.bicycle` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.collision_time_margin` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.traj_angle_threshold` | `0.523599` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_time_diff` | `0.35` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.min_velocity` | `-5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_velocity` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.position_diff` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.use_estimated_velocity` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.min_clamp_velocity` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.max_clamp_velocity` | `20.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.required_velocity_count` | `5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.lpf_gain` | `0.9` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_top` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_bottom` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/motion_velocity_planner/obstacle_stop.param.yaml` |
| `resample.ds_resample` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `resample.num_resample` | `1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `resample.delta_yaw_threshold` | `0.785` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `latacc.enable_constant_velocity_while_turning` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `latacc.constant_velocity_dist_threshold` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.min_acc` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.max_jerk` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.min_jerk` | `-0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `forward.kp` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.start_jerk` | `-0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_jerk_mild_stop` | `-0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_jerk` | `-1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_acc_mild_stop` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.min_acc` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `backward.span_jerk` | `-0.01` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Analytical.param.yaml` |
| `stop_decel` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `margin_to_insert_external_velocity_limit` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `curvature_calculation_distance` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `enable_lateral_acc_limit` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `lateral_acceleration_limits` | `[1.2, 1.0, 1.0, 1.0]` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `min_curve_velocity` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `decel_distance_before_curve` | `3.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `decel_distance_after_curve` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `min_decel_for_lateral_acc_lim_filter` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `enable_steering_rate_limit` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `velocity_thresholds` | `[0.1, 0.3, 20.0, 30.0]` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `steering_angle_rate_limits` | `[11.0, 11.0, 11.0, 11.0]` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `resample_ds` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `curvature_threshold` | `0.02` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `replan_vel_deviation` | `5.53` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `engage_velocity` | `0.25` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `engage_acceleration` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `engage_exit_ratio` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stop_dist_to_prohibit_engage` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stopping_velocity` | `2.778` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `stopping_distance` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `extract_ahead_dist` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `extract_behind_dist` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `delta_yaw_threshold` | `1.0472` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `max_trajectory_length` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `min_trajectory_length` | `180.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `resample_time` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `dense_resample_dt` | `0.2` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `dense_min_interval_distance` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `sparse_resample_dt` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `sparse_min_interval_distance` | `4.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_max_trajectory_length` | `300.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_resample_time` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_dense_resample_dt` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_dense_min_interval_distance` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_sparse_resample_dt` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `post_sparse_min_interval_distance` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `over_stop_velocity_warn_thr` | `1.389` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `plan_from_ego_speed_on_manual_mode` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `jerk_weight` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/JerkFiltered.param.yaml` |
| `over_v_weight` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/JerkFiltered.param.yaml` |
| `over_a_weight` | `5000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/JerkFiltered.param.yaml` |
| `over_j_weight` | `2000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/JerkFiltered.param.yaml` |
| `jerk_filter_ds` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/JerkFiltered.param.yaml` |
| `pseudo_jerk_weight` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Linf.param.yaml` |
| `over_v_weight` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Linf.param.yaml` |
| `over_a_weight` | `5000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/Linf.param.yaml` |
| `pseudo_jerk_weight` | `100.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/L2.param.yaml` |
| `over_v_weight` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/L2.param.yaml` |
| `over_a_weight` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/config/autoware_velocity_smoother/L2.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_stop_line_module`, `autoware_cmake`, `autoware_lint_common`, `autoware_mission_planner`, `autoware_motion_velocity_obstacle_stop_module`, `autoware_motion_velocity_planner`, `autoware_path_generator`, `autoware_planning_topic_converter`, `autoware_velocity_smoother`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 0 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 0 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/autoware_core_planning/README.md`
