> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_obstacle_stop_module

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module`
- Version: `1.8.0`
- Package description: obstacle stop feature in motion_velocity_planner
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::ObstacleStopModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |

## Purpose And Role

The `obstacle_stop` module does the stop planning when there is a static obstacle near the trajectory. The obstacles meeting the following condition are determined as obstacles for stopping. - The object type is for stopping according to `obstacle_filtering.object_type.*`. - The lateral distance from the object to the ego's trajectory is smaller than `obstacle_filtering.max_lat_margin`. - `obstacle_filtering.max_lat_margin_against_predicted_object_unknown` is applied to the predicted object of unknown.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/obstacle_stop/virtual_walls` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `~/obstacle_stop/debug_markers` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `~/debug/obstacle_stop/planning_info` | `Float32MultiArrayStamped` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `~/debug/processing_time_detail_ms/obstacle_stop` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `~/debug/obstacle_stop/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `obstacle_stop.option.ignore_crossing_obstacle` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `obstacle_stop.option.suppress_sudden_stop` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `obstacle_stop.stop_planning.update_distance_th` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `obstacle_stop.stop_planning.min_off_duration` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `obstacle_stop.stop_planning.min_on_duration` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp` |
| `normal.max_acc` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `normal.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `normal.max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `normal.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `limit.max_acc` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `limit.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `limit.max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `limit.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `<const std::string & ns>.{<object_label>|default}.<std::string suffix = "">` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.check_inside.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.check_outside.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.min_trajectory_length.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.braking_distance_scale_factor.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.wheel_off_track_scale.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_threshold_velocity.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_stop_obstacle.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_obstacle.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.detection_height.bottom_limit.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.min_object_length.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.min_velocity_to_reach_collision_point.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.stop_obstacle_hold_time_threshold.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.estimation_time_horizon.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_lateral_velocity.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.min_longitudinal_velocity.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_moving_direction_angle.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.collision_time_margin.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.traj_angle_threshold.{<object_label>|default}` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_time_diff` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.min_velocity` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_velocity` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.position_diff` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.use_estimated_velocity` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.min_clamp_velocity` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.max_clamp_velocity` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.required_velocity_count` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.lpf_gain` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_bottom` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_top` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.terminal_stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.min_behavior_stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.behavior_stop_margin_hold_time` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.max_negative_velocity` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.stop_margin_opposing_traffic` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.effective_deceleration_opposing_traffic` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.hold_stop_velocity_threshold` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.hold_stop_distance_threshold` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.pointcloud_suppression_distance_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.stop_on_curve.enable_approaching` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.stop_on_curve.additional_stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.stop_on_curve.min_stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.rss_params.use_rss_stop` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.rss_params.two_wheel_objects_deceleration` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.rss_params.vehicle_objects_deceleration` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.rss_params.no_wheel_objects_deceleration` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.rss_params.pointcloud_deceleration` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.rss_params.velocity_offset` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_enter_fixed_stop` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_exit_fixed_stop` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.object_type_specified_params.types` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.object_type_specified_params.<type_str>.limit_min_acc` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.object_type_specified_params.<type_str>.sudden_object_acc_threshold` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.object_type_specified_params.<type_str>.sudden_object_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |
| `obstacle_stop.stop_planning.object_type_specified_params.<type_str>.abandon_to_stop` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/src/parameters.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | parameter | `obstacle_stop` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.car` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.default` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.car` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.default` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.default` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.value` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.pedestrian.value` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.{<object_label>|default}` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.{<object_label>|default}` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.{<object_label>|default}` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.{<object_label>|default}` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `obstacle_stop.{<object_label>|default}.value` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_parameters.cpp` |
| test | parameter | `normal.min_acc` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `normal.max_acc` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `normal.min_jerk` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `normal.max_jerk` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `limit.min_acc` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `limit.max_acc` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `limit.min_jerk` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `limit.max_jerk` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |
| test | parameter | `pointcloud.mask_lat_margin` | `` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/test/test_obstacle_stop_module_outside_check.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `obstacle_stop.option.ignore_crossing_obstacle` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.option.suppress_sudden_stop` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_margin` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.terminal_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_behavior_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.behavior_stop_margin_hold_time` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.max_negative_velocity` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_margin_opposing_traffic` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.effective_deceleration_opposing_traffic` | `4.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_on_duration` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.min_off_duration` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.update_distance_th` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.hold_stop_velocity_threshold` | `0.01` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.hold_stop_distance_threshold` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.pointcloud_suppression_distance_margin` | `0.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.enable_approaching` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.additional_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.stop_on_curve.min_stop_margin` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.limit_min_acc` | `-1.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.sudden_object_acc_threshold` | `-1.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.sudden_object_dist_threshold` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.object_type_specified_params.unknown.abandon_to_stop` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.use_rss_stop` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.two_wheel_objects_deceleration` | `-3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.vehicle_objects_deceleration` | `-1.2` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.no_wheel_objects_deceleration` | `-10.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.pointcloud_deceleration` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.rss_params.velocity_offset` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_enter_fixed_stop` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.stop_planning.obstacle_velocity_threshold_exit_fixed_stop` | `3.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_inside.default` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_inside.pointcloud` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_outside.default` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.check_outside.unknown` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.default` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.enable_trimming.pointcloud` | `true` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.trim_trajectory.braking_distance_scale_factor` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.default` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.unknown` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.nominal.pointcloud` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.wheel_off_track_scale.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_threshold_velocity` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_stop_obstacle.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.lateral_margin.additional.is_moving_obstacle.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.default` | `50.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.top_limit.unknown` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.detection_height.bottom_limit` | `-50.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.min_object_length` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.min_velocity_to_reach_collision_point` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.stop_obstacle_hold_time_threshold` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.estimation_time_horizon` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_lateral_velocity` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.min_longitudinal_velocity` | `-8.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.max_moving_direction_angle` | `2.44` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.default` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.pedestrian` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.outside_obstacle.deceleration.bicycle` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.collision_time_margin` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.obstacle_filtering.crossing_obstacle.traj_angle_threshold` | `0.523599` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_time_diff` | `0.35` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.min_velocity` | `-5.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.max_velocity` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.time_series_association.position_diff` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.use_estimated_velocity` | `false` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.min_clamp_velocity` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.max_clamp_velocity` | `20.0` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.required_velocity_count` | `5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.velocity_estimation.lpf_gain` | `0.9` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_top` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |
| `obstacle_stop.pointcloud_segmentation.height_margin.margin_from_bottom` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_gtest`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_signal_processing`, `autoware_test_utils`, `autoware_utils_debug`, `autoware_utils_geometry`, `autoware_utils_rclcpp`, `autoware_utils_system`, `autoware_utils_uuid`, `autoware_utils_visualization`, `autoware_vehicle_info_utils`, `geometry_msgs`, `grid_map_core`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 65 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 2 |
| `pluginlib` | 3 |
| `parameters` | 71 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/README.md`
