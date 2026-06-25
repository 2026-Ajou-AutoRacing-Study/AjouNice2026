> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_traffic_light_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_traffic_light_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::TrafficLightModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::TrafficLightModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |

## Purpose And Role

Judgement whether a vehicle can go into an intersection or not by traffic light status, and planning a velocity of the stop if necessary. This module is designed for rule-based velocity decision that is easy for developers to design its behavior. It generates proper velocity for traffic light scene. This module allows developers to design STOP/GO in traffic light module using specific rules. Due to the property of rule-based planning, the algorithm is greatly depends on object detection and perception accuracy considering traffic light. Also, this module only handles STOP/Go at traffic light scene, so rushing or quick decision according to traffic condition is future work.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `~/output/traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `traffic_light.stop_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.tl_state_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.stop_time_hysteresis` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.enable_pass_judge` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.yellow_lamp_period` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.yellow_light_stop_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.enable_arrow_aware_yellow_passing` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.restart_suppression.min_behind_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.restart_suppression.max_behind_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.v2i.use_remaining_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.v2i.last_time_allowed_to_pass` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.v2i.velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.v2i.required_time_to_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/manager.cpp` |
| `traffic_light.stop_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.tl_state_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.stop_time_hysteresis` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.enable_pass_judge` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.yellow_lamp_period` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.yellow_light_stop_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.enable_arrow_aware_yellow_passing` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.restart_suppression.min_behind_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.restart_suppression.max_behind_distance_to_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.v2i.use_remaining_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.v2i.last_time_allowed_to_pass` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.v2i.velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |
| `traffic_light.v2i.required_time_to_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `traffic_light.stop_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.tl_state_timeout` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.stop_time_hysteresis` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.yellow_lamp_period` | `2.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.yellow_light_stop_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.enable_pass_judge` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.enable_arrow_aware_yellow_passing` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.restart_suppression.min_behind_distance_to_stop` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.restart_suppression.max_behind_distance_to_stop` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.v2i.use_remaining_time` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.v2i.last_time_allowed_to_pass` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.v2i.velocity_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |
| `traffic_light.v2i.required_time_to_departure` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/config/traffic_light.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_adapi_v1_msgs`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_test_utils`, `autoware_traffic_light_utils`, `autoware_utils`, `autoware_utils_geometry`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `pluginlib`, `rclcpp`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 47 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 3 |
| `pluginlib` | 5 |
| `parameters` | 27 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/README.md`
