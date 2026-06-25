> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_blind_spot_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_blind_spot_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::BlindSpotModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::BlindSpotModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/experimental/manager.cpp` |

## Purpose And Role

Blind spot module checks possible collisions with bicycles and pedestrians running on its left/right side while turing left/right before junctions. This function is activated when the lane id of the target path has an intersection label (i.e. the `turn_direction` attribute is `left` or `right`). Sets a stop line, a pass judge line, a detection area and conflict area based on a map information and a self position. - Stop line : Automatically created based on crossing lane information.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/blind_spot/decision_state` | `std_msgs::msg::String` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/manager.cpp` |
| `~/debug/blind_spot/decision_state` | `std_msgs::msg::String` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/experimental/manager.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `<ns>.backward_attention_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.ttc_start_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.ttc_end_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.minimum_default_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.collision_judge_debounce` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.critical_stopline_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.detect_non_vru.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.detect_non_vru.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.detect_non_vru.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.detect_non_vru.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.brake.critical.deceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.brake.critical.jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.brake.semi_critical.deceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.brake.semi_critical.jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.brake_for_ttc.critical_threshold_ub` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |
| `<ns>.brake_for_ttc.semi_critical_threshold_lb` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/src/parameter.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `blind_spot.backward_attention_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.ttc_start_margin` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.ttc_end_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.minimum_default_velocity` | `1.388` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.collision_judge_debounce` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.critical_stopline_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.detect_non_vru.car` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.detect_non_vru.truck` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.detect_non_vru.bus` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.detect_non_vru.trailer` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.brake.critical.deceleration` | `-6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.brake.critical.jerk` | `-5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.brake.semi_critical.deceleration` | `-4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.brake.semi_critical.jerk` | `-5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.brake_for_ttc.critical_threshold_ub` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.brake_for_ttc.semi_critical_threshold_lb` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |
| `blind_spot.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/config/blind_spot.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_test_utils`, `autoware_utils`, `geometry_msgs`, `pluginlib`, `range-v3`, `rclcpp`, `tf2`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 46 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 2 |
| `pluginlib` | 5 |
| `parameters` | 17 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/README.md`
