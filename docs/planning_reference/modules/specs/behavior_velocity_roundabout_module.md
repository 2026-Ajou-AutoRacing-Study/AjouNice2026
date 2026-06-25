> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_roundabout_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_roundabout_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::RoundaboutModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::RoundaboutModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |

## Purpose And Role

This module is responsible for safely managing entry into roundabouts by performing collision checks against vehicles in the attention area just before entry. Currently, it is designed to work with single-lane roundabouts due to the complexity of multi-lane scenarios. - A module instance is launched on a lane that is an entry lanelet of a Roundabout regulatory element on the path. - The HDMap needs to have the roundabout regulatory element defined with correct lanelet topology (entry/exit/inner lanes).

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/roundabout/decision_state` | `std_msgs::msg::String` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `~/debug/roundabout/ego_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/scene_roundabout.cpp` |
| `~/debug/roundabout/object_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/scene_roundabout.cpp` |
| `~/debug/roundabout/decision_state` | `std_msgs::msg::String` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `~/debug/roundabout/ego_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/scene_roundabout.cpp` |
| `~/debug/roundabout/object_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/scene_roundabout.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `roundabout.common.attention_area_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.common.attention_area_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.common.default_stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.common.path_interpolation_ds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.common.enable_pass_judge_before_default_stopline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.collision_detection_hold_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.min_predicted_path_confidence` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.use_upstream` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.minimum_upstream_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.minimum_default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.debug.ttc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/manager.cpp` |
| `roundabout.common.attention_area_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.common.attention_area_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.common.default_stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.common.path_interpolation_ds` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.common.enable_pass_judge_before_default_stopline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.collision_detection_hold_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.min_predicted_path_confidence` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.collision_start_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.collision_end_margin_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.target_type.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.use_upstream` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.minimum_upstream_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.velocity_profile.minimum_default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |
| `roundabout.debug.ttc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `roundabout.common.attention_area_margin` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.common.attention_area_angle_threshold` | `0.785` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.common.default_stopline_margin` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.common.path_interpolation_ds` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.common.enable_pass_judge_before_default_stopline` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.collision_detection_hold_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.min_predicted_path_confidence` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.collision_start_margin_time` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.collision_end_margin_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.target_type.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.velocity_profile.use_upstream` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.velocity_profile.minimum_upstream_velocity` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.velocity_profile.default_velocity` | `2.778` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.velocity_profile.minimum_default_velocity` | `1.388` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.debug.ttc` | `[0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |
| `roundabout.enable_rtc.roundabout` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/config/roundabout.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_behavior_velocity_intersection_module`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_rtc_interface`, `autoware_test_utils`, `autoware_utils`, `autoware_vehicle_info_utils`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `sensor_msgs`, `tf2`, `tf2_geometry_msgs`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 59 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 8 |
| `pluginlib` | 5 |
| `parameters` | 45 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/README.md`
