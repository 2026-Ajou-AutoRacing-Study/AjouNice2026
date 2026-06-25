> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_virtual_traffic_light_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_virtual_traffic_light_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::VirtualTrafficLightModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::VirtualTrafficLightModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |

## Purpose And Role

Autonomous vehicles have to cooperate with the infrastructures such as: - Warehouse shutters - Traffic lights with V2X support - Communication devices at intersections - Fleet Management Systems (FMS) The following items are example cases: 1. Traffic control by traffic lights with V2X support 2. Intersection coordination of multiple vehicles by FMS. It's possible to make each function individually, however, the use cases can be generalized with these three elements.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/infrastructure_commands` | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `~/output/infrastructure_commands` | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `virtual_traffic_light.max_delay_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `virtual_traffic_light.near_line_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `virtual_traffic_light.dead_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `virtual_traffic_light.hold_stop_margin_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `virtual_traffic_light.max_yaw_deviation_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `virtual_traffic_light.check_timeout_after_stop_line` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/manager.cpp` |
| `virtual_traffic_light.max_delay_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |
| `virtual_traffic_light.near_line_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |
| `virtual_traffic_light.dead_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |
| `virtual_traffic_light.hold_stop_margin_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |
| `virtual_traffic_light.max_yaw_deviation_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |
| `virtual_traffic_light.check_timeout_after_stop_line` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | publisher | `behavior_velocity_planner_node/input/virtual_traffic_light_states` | `VirtualTrafficLightStateArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/test/test_node_interface.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `virtual_traffic_light.max_delay_sec` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/config/virtual_traffic_light.param.yaml` |
| `virtual_traffic_light.near_line_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/config/virtual_traffic_light.param.yaml` |
| `virtual_traffic_light.dead_line_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/config/virtual_traffic_light.param.yaml` |
| `virtual_traffic_light.max_yaw_deviation_deg` | `90.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/config/virtual_traffic_light.param.yaml` |
| `virtual_traffic_light.check_timeout_after_stop_line` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/config/virtual_traffic_light.param.yaml` |
| `virtual_traffic_light.hold_stop_margin_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/config/virtual_traffic_light.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_adapi_v1_msgs`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils`, `autoware_utils_geometry`, `autoware_vehicle_info_utils`, `geometry_msgs`, `nlohmann-json-dev`, `pluginlib`, `rclcpp`, `tier4_planning_msgs`, `tier4_v2x_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 30 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 5 |
| `parameters` | 13 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/README.md`
