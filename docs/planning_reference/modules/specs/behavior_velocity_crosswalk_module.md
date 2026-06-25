> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_crosswalk_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_crosswalk_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::CrosswalkModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::CrosswalkModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |

## Purpose And Role

This module judges whether the ego should stop in front of the crosswalk in order to provide safe passage for crosswalk users, such as pedestrians and bicycles, based on the objects' behavior and surround traffic. The crosswalk module handles objects of the types defined by the following parameters in the `object_filtering.target_object` namespace. In order to handle the crosswalk users crossing the neighborhood but outside the crosswalk, the crosswalk module creates an attention area around the crosswalk, shown as the yellow polygon in the figure. If the object's predicted path collides with the attention area, the object will be targeted for yield.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/collision_info` | `autoware_internal_debug_msgs::msg::StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp` |
| `~/debug/collision_info` | `autoware_internal_debug_msgs::msg::StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/scene_crosswalk.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `crosswalk.common.show_processing_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.common.traffic_light_state_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.common.lost_detection_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.stop_position.stop_distance_from_crosswalk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.stop_position.stop_distance_from_object_preferred` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.stop_position.stop_distance_from_crosswalk_limit` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.stop_position.stop_position_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.stop_position.min_acc_preferred` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.stop_position.min_jerk_preferred` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.restart_suppression.min_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.restart_suppression.max_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.slow_down.min_slow_down_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.slow_down.max_slow_down_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.slow_down.max_slow_down_accel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.slow_down.no_relax_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.enable_obstruction_prevention` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.target_vehicle_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.required_lateral_clearance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.required_clearance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.obstruction_prevention.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_first_margin_x` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_first_margin_y` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_first_additional_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_later_margin_x` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_later_margin_y` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_later_additional_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.ego_min_assumed_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.consider_obj_on_crosswalk_on_red_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.no_stop_decision.enable_no_stop_decision` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.no_stop_decision.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.no_stop_decision.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.stop_object_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.min_object_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.disable_yield_for_new_stopped_object` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.distance_set_for_no_intention_to_walk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.timeout_set_for_no_intention_to_walk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.timeout_ego_stop_for_yield` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.pass_judge.min_vru_crossing_width` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.object_filtering.crosswalk_attention_range` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.object_filtering.vehicle_object_cross_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.object_filtering.target_object.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.object_filtering.target_object.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.object_filtering.target_object.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.object_filtering.target_object.pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.occluded_object_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.slow_down_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.time_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.min_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.free_space_max` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.occupied_min` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.ignore_with_traffic_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.ignore_behind_predicted_objects` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.ignore_velocity_thresholds.default` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.ignore_velocity_thresholds.custom_labels` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.ignore_velocity_thresholds.custom_thresholds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.occlusion.extra_predicted_objects_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.parked_vehicles_stop.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.parked_vehicles_stop.search_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.parked_vehicles_stop.min_ego_stop_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.parked_vehicles_stop.ego_inside_safe_area_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.parked_vehicles_stop.parked_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.parked_vehicles_stop.vehicle_permanence_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp` |
| `crosswalk.common.show_processing_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.common.traffic_light_state_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.common.lost_detection_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.stop_position.stop_distance_from_crosswalk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.stop_position.stop_distance_from_object_preferred` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.stop_position.stop_distance_from_crosswalk_limit` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.stop_position.stop_position_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.stop_position.min_acc_preferred` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.stop_position.min_jerk_preferred` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.restart_suppression.min_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.restart_suppression.max_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.slow_down.min_slow_down_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.slow_down.max_slow_down_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.slow_down.max_slow_down_accel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.slow_down.no_relax_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.enable_obstruction_prevention` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.target_vehicle_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.required_lateral_clearance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.required_clearance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.obstruction_prevention.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_first_margin_x` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_first_margin_y` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_first_additional_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_later_margin_x` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_later_margin_y` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_pass_later_additional_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.ego_min_assumed_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.consider_obj_on_crosswalk_on_red_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.no_stop_decision.enable_no_stop_decision` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.no_stop_decision.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.no_stop_decision.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.stop_object_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.min_object_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.disable_yield_for_new_stopped_object` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.distance_set_for_no_intention_to_walk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.timeout_set_for_no_intention_to_walk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.timeout_ego_stop_for_yield` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.pass_judge.min_vru_crossing_width` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.object_filtering.crosswalk_attention_range` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.object_filtering.vehicle_object_cross_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.object_filtering.target_object.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.object_filtering.target_object.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.object_filtering.target_object.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.object_filtering.target_object.pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.occluded_object_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.slow_down_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.time_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.min_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.free_space_max` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.occupied_min` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.ignore_with_traffic_light` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.ignore_behind_predicted_objects` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.ignore_velocity_thresholds.default` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.ignore_velocity_thresholds.custom_labels` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.ignore_velocity_thresholds.custom_thresholds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.occlusion.extra_predicted_objects_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.parked_vehicles_stop.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.parked_vehicles_stop.search_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.parked_vehicles_stop.min_ego_stop_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.parked_vehicles_stop.ego_inside_safe_area_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.parked_vehicles_stop.parked_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |
| `crosswalk.parked_vehicles_stop.vehicle_permanence_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `crosswalk.common.show_processing_time` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.common.traffic_light_state_timeout` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.common.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.common.lost_detection_timeout` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.stop_position.stop_position_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.stop_position.stop_distance_from_crosswalk` | `3.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.stop_position.stop_distance_from_object_preferred` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.stop_position.stop_distance_from_crosswalk_limit` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.stop_position.min_acc_preferred` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.stop_position.min_jerk_preferred` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.slow_down.min_slow_down_velocity` | `2.78` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.slow_down.max_slow_down_jerk` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.slow_down.max_slow_down_accel` | `-2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.slow_down.no_relax_velocity` | `2.78` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.enable_obstruction_prevention` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.target_vehicle_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.required_lateral_clearance` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.required_clearance` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.min_acc` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.min_jerk` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.obstruction_prevention.max_jerk` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_pass_first_margin_x` | `[0.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_pass_first_margin_y` | `[4.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_pass_first_additional_margin` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_pass_later_margin_x` | `[0.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_pass_later_margin_y` | `[13.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_pass_later_additional_margin` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.ego_min_assumed_speed` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.consider_obj_on_crosswalk_on_red_light` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.no_stop_decision.enable_no_stop_decision` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.no_stop_decision.min_acc` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.no_stop_decision.min_jerk` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.stop_object_velocity_threshold` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.min_object_velocity` | `1.39` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.disable_yield_for_new_stopped_object` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.distance_set_for_no_intention_to_walk` | `[1.0, 5.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.timeout_set_for_no_intention_to_walk` | `[1.0, 0.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.timeout_ego_stop_for_yield` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.pass_judge.min_vru_crossing_width` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.restart_suppression.min_distance_to_stop` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.restart_suppression.max_distance_to_stop` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.object_filtering.crosswalk_attention_range` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.object_filtering.vehicle_object_cross_angle_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.object_filtering.target_object.unknown` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.object_filtering.target_object.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.object_filtering.target_object.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.object_filtering.target_object.pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.occluded_object_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.slow_down_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.time_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.min_size` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.free_space_max` | `43` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.occupied_min` | `58` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.ignore_with_traffic_light` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.ignore_behind_predicted_objects` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.ignore_velocity_thresholds.default` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.ignore_velocity_thresholds.custom_labels` | `["PEDESTRIAN"]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.ignore_velocity_thresholds.custom_thresholds` | `[0.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.occlusion.extra_predicted_objects_size` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.parked_vehicles_stop.enable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.parked_vehicles_stop.search_distance` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.parked_vehicles_stop.min_ego_stop_duration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.parked_vehicles_stop.vehicle_permanence_duration` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.parked_vehicles_stop.ego_inside_safe_area_margin` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |
| `crosswalk.parked_vehicles_stop.parked_velocity_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/config/crosswalk.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_grid_map_utils`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_signal_processing`, `autoware_utils`, `autoware_vehicle_info_utils`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `grid_map_core`, `grid_map_ros`, `libboost-dev`, `nav_msgs`, `pcl_conversions`, `pluginlib`, `rclcpp`, `sensor_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 72 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 5 |
| `parameters` | 131 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/README.md`
