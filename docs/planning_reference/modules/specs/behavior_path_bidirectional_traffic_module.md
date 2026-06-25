> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_bidirectional_traffic_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_bidirectional_traffic_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::BidirectionalTrafficModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/src/manager.cpp` |

## Purpose And Role

This module enables trajectory planning for vehicles on "single-lane bidirectional traffic" roads in autonomous driving systems. Here, "single-lane bidirectional traffic" refers to roads without a centerline, such as those commonly found in residential areas, as shown in the image below． In the following descriptions, the term bidirectional traffic refers specifically to such single-lane bidirectional roads as shown in the image above. The main roles of this module are as follows:

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
| `bidirectional_traffic.time_to_prepare_pull_over` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |
| `bidirectional_traffic.min_distance_from_roadside` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |
| `bidirectional_traffic.keep_left_distance_from_center_line` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |
| `bidirectional_traffic.shift_distance_to_pull_over_from_center_line` | `1.6` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |
| `bidirectional_traffic.wait_time_for_oncoming_car` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |
| `bidirectional_traffic.max_lateral_jerk` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |
| `bidirectional_traffic.min_lateral_jerk` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/config/bidirectional_traffic.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_cmake`, `autoware_lint_common`, `autoware_test_utils`, `autoware_trajectory`, `autoware_utils`, `autoware_utils_geometry`, `eigen3_cmake_module`, `fmt`, `lanelet2_core`, `pluginlib`, `rclcpp`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 21 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 6 |
| `pluginlib` | 3 |
| `parameters` | 7 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/README.md`
