> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_intersection_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_intersection_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::IntersectionModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::MergeFromPrivateModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::IntersectionModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::MergeFromPrivateModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |

## Purpose And Role

The intersection module is responsible for safely passing urban intersections by: 1. checking collisions with upcoming vehicles 2. recognizing the occluded area in the intersection 3. reacting to each color/shape of associated traffic lights This module is designed to be agnostic to left-hand/right-hand traffic rules and work for crossroads, T-shape junctions, etc. Roundabout is not formally supported in this module. This module is activated when the path contains the lanes with turn_direction tag. More precisely, if the lane_ids of the path contain the ids of those lanes, corresponding instances of intersection module are activated on each lane respectively.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/intersection/ego_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/scene_intersection.cpp` |
| `~/debug/intersection/object_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/scene_intersection.cpp` |
| `~/debug/intersection/decision_state` | `std_msgs::msg::String` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `~/debug/intersection_traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `~/debug/intersection/ego_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/scene_intersection.cpp` |
| `~/debug/intersection/object_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/scene_intersection.cpp` |
| `~/debug/intersection/decision_state` | `std_msgs::msg::String` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `~/debug/intersection_traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `intersection.common.attention_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.path_interpolation_ds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.attention_area_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.attention_area_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.use_intersection_area` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.default_stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.pass_judge_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.stopline_overshoot_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.max_accel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.delay_response_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.turn_direction.left` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.turn_direction.right` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.turn_direction.straight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.use_stuck_stopline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.stuck_vehicle_detect_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.stuck_vehicle_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.stuck_vehicle.disable_against_private_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.turn_direction.left` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.turn_direction.right` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.turn_direction.straight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.yield_stuck.distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.consider_wrong_direction_vehicle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.collision_detection_hold_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.min_predicted_path_confidence` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.velocity_profile.use_upstream` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.velocity_profile.minimum_upstream_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.velocity_profile.default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.velocity_profile.minimum_default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.fully_prioritized.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.fully_prioritized.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.partially_prioritized.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.partially_prioritized.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.not_prioritized.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.not_prioritized.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.yield_on_green_traffic_light.duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.yield_on_green_traffic_light.object_dist_to_stopline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.bike` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.ignore_on_red_traffic_light.object_margin_to_path` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.request_approval_wo_traffic_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.occlusion_attention_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.free_space_max` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.occupied_min` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.denoise_kernel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.attention_lane_crop_curvature_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.attention_lane_curvature_calculation_ds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.peeking_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.occlusion_required_clearance_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.possible_object_bbox` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.ignore_parked_vehicle_speed_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.occlusion_detection_hold_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.temporal_stop_time_before_peeking` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.creep_velocity_without_traffic_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.occlusion.static_occlusion_with_traffic_light_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.conservative_merging.enable_yield` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.conservative_merging.minimum_lateral_distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.conservative_merging.merging_judge_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.debug.ttc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `planning_factor_console_output.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `planning_factor_console_output.duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `merge_from_private.stop_duration_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `merge_from_private.stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `merge_from_private.stop_distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/manager.cpp` |
| `intersection.common.attention_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.path_interpolation_ds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.attention_area_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.attention_area_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.use_intersection_area` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.default_stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.pass_judge_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.stopline_overshoot_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.max_accel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.common.delay_response_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.turn_direction.left` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.turn_direction.right` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.turn_direction.straight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.use_stuck_stopline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.stuck_vehicle_detect_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.stuck_vehicle_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.stuck_vehicle.disable_against_private_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.turn_direction.left` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.turn_direction.right` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.turn_direction.straight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.yield_stuck.distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.consider_wrong_direction_vehicle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.collision_detection_hold_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.min_predicted_path_confidence` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.velocity_profile.use_upstream` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.velocity_profile.minimum_upstream_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.velocity_profile.default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.velocity_profile.minimum_default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.fully_prioritized.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.fully_prioritized.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.partially_prioritized.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.partially_prioritized.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.not_prioritized.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.not_prioritized.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.yield_on_green_traffic_light.duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.yield_on_green_traffic_light.object_dist_to_stopline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.bike` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.ignore_on_red_traffic_light.object_margin_to_path` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.request_approval_wo_traffic_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.occlusion_attention_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.free_space_max` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.occupied_min` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.denoise_kernel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.attention_lane_crop_curvature_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.attention_lane_curvature_calculation_ds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.peeking_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.occlusion_required_clearance_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.possible_object_bbox` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.ignore_parked_vehicle_speed_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.occlusion_detection_hold_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.temporal_stop_time_before_peeking` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.creep_velocity_without_traffic_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.occlusion.static_occlusion_with_traffic_light_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.conservative_merging.enable_yield` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.conservative_merging.minimum_lateral_distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.conservative_merging.merging_judge_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `intersection.debug.ttc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `planning_factor_console_output.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `planning_factor_console_output.duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `merge_from_private.stop_duration_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `merge_from_private.stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |
| `merge_from_private.stop_distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `intersection.common.attention_area_length` | `200.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.attention_area_margin` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.attention_area_angle_threshold` | `0.785` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.use_intersection_area` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.default_stopline_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.pass_judge_line_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.stopline_overshoot_margin` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.path_interpolation_ds` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.max_accel` | `-2.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.max_jerk` | `-5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.common.delay_response_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.motorcycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.bicycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.target_type.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.turn_direction.left` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.turn_direction.right` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.turn_direction.straight` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.use_stuck_stopline` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.stuck_vehicle_detect_dist` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.stuck_vehicle_velocity_threshold` | `0.833` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.stuck_vehicle.disable_against_private_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.motorcycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.bicycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.target_type.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.turn_direction.left` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.turn_direction.right` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.turn_direction.straight` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.yield_stuck.distance_threshold` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.consider_wrong_direction_vehicle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.collision_detection_hold_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.min_predicted_path_confidence` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.target_type.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.velocity_profile.use_upstream` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.velocity_profile.minimum_upstream_velocity` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.velocity_profile.default_velocity` | `2.778` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.velocity_profile.minimum_default_velocity` | `1.388` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.fully_prioritized.collision_start_margin_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.fully_prioritized.collision_end_margin_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.partially_prioritized.collision_start_margin_time` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.partially_prioritized.collision_end_margin_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.not_prioritized.collision_start_margin_time` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.not_prioritized.collision_end_margin_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.yield_on_green_traffic_light.duration` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.yield_on_green_traffic_light.object_dist_to_stopline` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.car` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration.bike` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.ignore_on_red_traffic_light.object_margin_to_path` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.request_approval_wo_traffic_light` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.occlusion_attention_area_length` | `70.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.free_space_max` | `43` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.occupied_min` | `58` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.denoise_kernel` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.attention_lane_crop_curvature_threshold` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.attention_lane_curvature_calculation_ds` | `0.6` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.peeking_offset` | `-0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.occlusion_required_clearance_distance` | `55.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.possible_object_bbox` | `[1.9, 2.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.ignore_parked_vehicle_speed_threshold` | `0.8333` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.occlusion_detection_hold_time` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.temporal_stop_time_before_peeking` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.creep_velocity_without_traffic_light` | `1.388` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.occlusion.static_occlusion_with_traffic_light_timeout` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.conservative_merging.enable_yield` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.conservative_merging.minimum_lateral_distance_threshold` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.conservative_merging.merging_judge_angle_threshold` | `0.785` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.debug.ttc` | `[0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.enable_rtc.intersection` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `intersection.enable_rtc.intersection_to_occlusion` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `merge_from_private.stopline_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `merge_from_private.stop_duration_sec` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |
| `merge_from_private.stop_distance_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/config/intersection.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_rtc_interface`, `autoware_test_utils`, `autoware_vehicle_info_utils`, `fmt`, `geometry_msgs`, `libopencv-dev`, `magic_enum`, `nav_msgs`, `pluginlib`, `rclcpp`, `tf2_geometry_msgs`, `tier4_planning_msgs`, `visualization_msgs`, `yaml-cpp`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 125 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 14 |
| `pluginlib` | 7 |
| `parameters` | 181 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/README.md`
