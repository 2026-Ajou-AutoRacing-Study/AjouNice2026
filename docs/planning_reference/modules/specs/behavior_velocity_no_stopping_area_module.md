> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_no_stopping_area_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_no_stopping_area_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::NoStoppingAreaModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::NoStoppingAreaModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |

## Purpose And Role

This module plans to avoid stop in 'no stopping area`. - PassThrough case - if ego vehicle go through pass judge point, then ego vehicle can't stop with maximum jerk and acceleration, so this module doesn't insert stop velocity. In this case override or external operation is necessary. - STOP case - If there is a stuck vehicle or stop velocity around `no_stopping_area`, then vehicle stops inside `no_stopping_area` so this module makes stop velocity in front of `no_stopping_area`

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
| `no_stopping_area.state_clear_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.stuck_vehicle_vel_thr` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.stop_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.dead_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.stop_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.detection_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.stuck_vehicle_front_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/manager.cpp` |
| `no_stopping_area.state_clear_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |
| `no_stopping_area.stuck_vehicle_vel_thr` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |
| `no_stopping_area.stop_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |
| `no_stopping_area.dead_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |
| `no_stopping_area.stop_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |
| `no_stopping_area.detection_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |
| `no_stopping_area.stuck_vehicle_front_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `no_stopping_area.state_clear_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.stuck_vehicle_vel_thr` | `0.833` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.stop_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.dead_line_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.stop_line_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.detection_area_length` | `200.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.stuck_vehicle_front_margin` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |
| `no_stopping_area.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/config/no_stopping_area.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 57 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 1 |
| `pluginlib` | 5 |
| `parameters` | 15 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/README.md`
