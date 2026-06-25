> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_lane_change_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_lane_change_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::LaneChangeRightModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_path_planner::LaneChangeLeftModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |

## Purpose And Role

The Lane Change module is activated when lane change is needed (Ego is not on preferred lane), and activation requirements are satisfied. The type of lane boundary in the HD map has to be one of the following: - Dashed lane marking: Lane changes are permitted in both directions. - Dashed marking on the left and solid on the right: Lane changes are allowed from left to right. - Dashed marking on the right and solid on the left: Lane changes are allowed from right to left.

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
| `<parameter("trajectory.max_prepare_duration")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.min_prepare_duration")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.lateral_jerk")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.min_longitudinal_acc")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.max_longitudinal_acc")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.th_prepare_length_diff")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.th_lane_changing_length_diff")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.min_lane_changing_velocity")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.lane_changing_decel_factor")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.th_prepare_curvature")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.lon_acc_sampling_num")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("trajectory.lat_acc_sampling_num")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `normal.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("lateral_acceleration.velocity")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("lateral_acceleration.min_values")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("lateral_acceleration.max_values")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("min_length_for_turn_signal_activation")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("regulation.crosswalk")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("regulation.intersection")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("regulation.traffic_light")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("stuck_detection.velocity")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("stuck_detection.stop_time")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("safety_check.allow_loose_check_for_cancel")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("safety_check.enable_target_lane_bound_check")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("safety_check.stopped_object_velocity_threshold")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("safety_check.lane_expansion.left_offset")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("safety_check.lane_expansion.right_offset")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("collision_check.check_current_lanes")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("collision_check.check_other_lanes")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("collision_check.use_all_predicted_paths")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("collision_check.prediction_time_resolution")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("collision_check.yaw_diff_threshold")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("collision_check.th_incoming_object_yaw")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".longitudinal_distance_min_threshold")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".longitudinal_velocity_delta_time")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".expected_front_deceleration")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".expected_rear_deceleration")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".rear_vehicle_reaction_time")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".rear_vehicle_safety_time_margin")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".lateral_distance_max_threshold")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter(prefix><".extended_polygon_policy")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `lane_change.target_object.pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("time_limit")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("backward_lane_length")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("backward_length_buffer_for_end_of_lane")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("backward_length_buffer_for_blocking_object")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("backward_length_from_intersection")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("enable_stopped_vehicle_buffer")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("delay_lane_change.enable")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("delay_lane_change.check_only_parked_vehicle")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("delay_lane_change.min_road_shoulder_width")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("delay_lane_change.th_parked_vehicle_shift_ratio")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("frenet.enable")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("frenet.use_entire_remaining_distance")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("frenet.th_yaw_diff")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("frenet.th_curvature_smoothing")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("frenet.th_average_curvature")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.enable_on_prepare_phase")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.enable_on_lane_changing_phase")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.delta_time")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.duration")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.max_lateral_jerk")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.overhang_tolerance")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.unsafe_hysteresis_threshold")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("cancel.deceleration_sampling_num")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("lane_change_finish_judge_buffer")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("finish_judge_lateral_threshold")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("finish_judge_lateral_angle_deviation")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("path_miss.enable_path_miss_detection")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("path_miss.threshold_longitudinal")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("path_miss.velocity_points")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("path_miss.lateral_thresholds")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("publish_debug_marker")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("terminal_path.enable")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("terminal_path.disable_near_goal")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |
| `<parameter("terminal_path.stop_at_boundary")>` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `lane_change.time_limit` | `50.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.backward_lane_length` | `200.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.backward_length_buffer_for_end_of_lane` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.backward_length_buffer_for_blocking_object` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.backward_length_from_intersection` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.enable_stopped_vehicle_buffer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.min_length_for_turn_signal_activation` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.terminal_path.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.terminal_path.disable_near_goal` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.terminal_path.stop_at_boundary` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.max_prepare_duration` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.min_prepare_duration` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.lateral_jerk` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.min_longitudinal_acc` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.max_longitudinal_acc` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.th_prepare_length_diff` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.th_lane_changing_length_diff` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.min_lane_changing_velocity` | `2.78` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.lon_acc_sampling_num` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.lat_acc_sampling_num` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.lane_changing_decel_factor` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.trajectory.th_prepare_curvature` | `0.03` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.delay_lane_change.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.delay_lane_change.check_only_parked_vehicle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.delay_lane_change.min_road_shoulder_width` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.delay_lane_change.th_parked_vehicle_shift_ratio` | `0.6` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.frenet.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.frenet.use_entire_remaining_distance` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.frenet.th_yaw_diff` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.frenet.th_curvature_smoothing` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.frenet.th_average_curvature` | `0.015` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.allow_loose_check_for_cancel` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.enable_target_lane_bound_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stopped_object_velocity_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.expected_front_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.expected_rear_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.rear_vehicle_reaction_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.rear_vehicle_safety_time_margin` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.lateral_distance_max_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.longitudinal_distance_min_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.longitudinal_velocity_delta_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.prepare.extended_polygon_policy` | `"along_path"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.expected_front_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.expected_rear_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.rear_vehicle_reaction_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.rear_vehicle_safety_time_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.lateral_distance_max_threshold` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.longitudinal_distance_min_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.longitudinal_velocity_delta_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.execution.extended_polygon_policy` | `"rectangle"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.expected_front_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.expected_rear_deceleration` | `-2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.rear_vehicle_reaction_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.rear_vehicle_safety_time_margin` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.lateral_distance_max_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.longitudinal_distance_min_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.longitudinal_velocity_delta_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.parked.extended_polygon_policy` | `"rectangle"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.expected_front_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.expected_rear_deceleration` | `-2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.rear_vehicle_reaction_time` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.rear_vehicle_safety_time_margin` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.lateral_distance_max_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.longitudinal_distance_min_threshold` | `2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.longitudinal_velocity_delta_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.cancel.extended_polygon_policy` | `"rectangle"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.expected_front_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.expected_rear_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.rear_vehicle_reaction_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.rear_vehicle_safety_time_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.lateral_distance_max_threshold` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.longitudinal_distance_min_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.longitudinal_velocity_delta_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.stuck.extended_polygon_policy` | `"rectangle"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.lane_expansion.left_offset` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.safety_check.lane_expansion.right_offset` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.lateral_acceleration.velocity` | `[0.0, 4.0, 10.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.lateral_acceleration.min_values` | `[0.4,0.4,0.4]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.lateral_acceleration.max_values` | `[0.65,0.65,0.65]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.target_object.pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.regulation.crosswalk` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.regulation.intersection` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.regulation.traffic_light` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.stuck_detection.velocity` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.stuck_detection.stop_time` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.collision_check.prediction_time_resolution` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.collision_check.th_incoming_object_yaw` | `2.3562` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.collision_check.yaw_diff_threshold` | `3.1416` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.collision_check.check_current_lanes` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.collision_check.check_other_lanes` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.collision_check.use_all_predicted_paths` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.enable_on_prepare_phase` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.enable_on_lane_changing_phase` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.delta_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.duration` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.max_lateral_jerk` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.overhang_tolerance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.unsafe_hysteresis_threshold` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.cancel.deceleration_sampling_num` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.lane_change_finish_judge_buffer` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.finish_judge_lateral_threshold` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.finish_judge_lateral_angle_deviation` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.path_miss.enable_path_miss_detection` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.path_miss.threshold_longitudinal` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.path_miss.velocity_points` | `[ 0.0, 0.1, 0.11, 2.77, 2.78, 20.0 ]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.path_miss.lateral_thresholds` | `[ 3.0, 3.0, 5.0,  5.0,  1.5,   0.8 ]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |
| `lane_change.publish_debug_marker` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_cmake`, `autoware_frenet_planner`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_rtc_interface`, `autoware_utils`, `eigen3_cmake_module`, `fmt`, `pluginlib`, `range-v3`, `rclcpp`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 57 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 5 |
| `pluginlib` | 4 |
| `parameters` | 84 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md`
