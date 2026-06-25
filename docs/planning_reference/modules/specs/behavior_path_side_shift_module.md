> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_side_shift_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_side_shift_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::SideShiftModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/src/manager.cpp` |

## Purpose And Role

(For remote control) Shift the path to left or right according to an external instruction. 1. Receive the required lateral offset input. 2. Update the `requested_lateral_offset_` under the following conditions: a. Verify if the last update time has elapsed. b. Ensure the required lateral offset value is different from the previous one. 3. Insert the shift points into the path if the side shift module's status is not in the SHIFTING status. Please be aware that `requested_lateral_offset_` is continuously updated with the latest values and is not queued.

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
| _none detected by literal scan_ |  |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `side_shift.min_distance_to_start_shifting` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |
| `side_shift.time_to_start_shifting` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |
| `side_shift.shifting_lateral_jerk` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |
| `side_shift.min_shifting_distance` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |
| `side_shift.min_shifting_speed` | `5.56` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |
| `side_shift.shift_request_time_limit` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |
| `side_shift.publish_debug_marker` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/config/side_shift.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_utils`, `eigen3_cmake_module`, `geometry_msgs`, `pluginlib`, `rclcpp`, `tier4_planning_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 22 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 9 |
| `pluginlib` | 3 |
| `parameters` | 7 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module/README.md`
