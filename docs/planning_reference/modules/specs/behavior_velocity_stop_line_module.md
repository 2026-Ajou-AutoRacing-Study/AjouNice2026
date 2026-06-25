> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_stop_line_module

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module`
- Version: `1.8.0`
- Package description: The autoware_behavior_velocity_stop_line_module package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::StopLineModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::StopLineModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/experimental/manager.cpp` |

## Purpose And Role

This module plans the vehicle's velocity to ensure it stops just before stop lines and can resume movement after stopping. This module is activated when there is a stop line in a target lane. - Gets a stop line from map information. - Inserts a stop point on the path from the stop line defined in the map and the ego vehicle length. - Sets velocities of the path after the stop point to 0[m/s]. - Releases the inserted stop velocity when the vehicle halts at the stop point for `required_stop_duration_sec` seconds.

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
| `stop_line.stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp` |
| `stop_line.hold_stop_margin_distance` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp` |
| `stop_line.required_stop_duration_sec` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp` |
| `stop_line.vehicle_stopped_duration_threshold` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp` |
| `stop_line.stop_margin` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/experimental/manager.cpp` |
| `stop_line.hold_stop_margin_distance` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/experimental/manager.cpp` |
| `stop_line.required_stop_duration_sec` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/experimental/manager.cpp` |
| `stop_line.vehicle_stopped_duration_threshold` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `stop_line.stop_margin` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/config/stop_line.param.yaml` |
| `stop_line.required_stop_duration_sec` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/config/stop_line.param.yaml` |
| `stop_line.vehicle_stopped_duration_threshold` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/config/stop_line.param.yaml` |
| `stop_line.hold_stop_margin_distance` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/config/stop_line.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_route_handler`, `autoware_trajectory`, `autoware_utils_debug`, `autoware_utils_geometry`, `autoware_utils_rclcpp`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `pluginlib`, `rclcpp`, `tf2_geometry_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 47 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 5 |
| `parameters` | 9 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/README.md`
