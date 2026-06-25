> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_speed_bump_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_speed_bump_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::SpeedBumpModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |

## Purpose And Role

This module plans the velocity of the related part of the path in case there is speed bump regulatory element referring to it. The manager launch speed bump scene module when there is speed bump regulatory element referring to the reference path. - limits for speed bump height and slow down speed to create a linear equation - Get speed bump regulatory element on the path from lanelet2 map - Calculate `slow_down_speed` wrt to `speed_bump_height` specified in regulatory element or

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
| `speed_bump.slow_start_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.slow_end_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.print_debug_info` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.min_height` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.max_height` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.min_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.max_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/manager.cpp` |
| `speed_bump.slow_start_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |
| `speed_bump.slow_end_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |
| `speed_bump.print_debug_info` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |
| `speed_bump.min_height` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |
| `speed_bump.max_height` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |
| `speed_bump.min_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |
| `speed_bump.max_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `speed_bump.slow_start_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |
| `speed_bump.slow_end_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |
| `speed_bump.print_debug_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |
| `speed_bump.speed_calculation.min_height` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |
| `speed_bump.speed_calculation.max_height` | `0.30` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |
| `speed_bump.speed_calculation.min_speed` | `1.39` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |
| `speed_bump.speed_calculation.max_speed` | `2.78` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/config/speed_bump.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_behavior_velocity_planner_common`, `autoware_cmake`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils`, `autoware_utils_geometry`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `sensor_msgs`, `tf2`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 23 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 2 |
| `pluginlib` | 5 |
| `parameters` | 15 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/README.md`
