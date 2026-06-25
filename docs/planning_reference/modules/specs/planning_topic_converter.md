> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_topic_converter

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/autoware_planning_topic_converter`
- Version: `1.8.0`
- Package description: The autoware_planning_topic_converter package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/autoware_planning_topic_converter/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::planning_topic_converter::PathToTrajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_planning_topic_converter/src/path_to_trajectory.cpp` |

## Purpose And Role

This package provides tools that convert topic type among types are defined in <https://github.com/autowarefoundation/autoware_msgs>. The tools in this package are provided as composable ROS 2 component nodes, so that they can be spawned into an existing process, launched from launch files, or invoked from the command line.

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
| `input_topic` | `sources/autoware_workspace/core/autoware_core/planning/autoware_planning_topic_converter/include/autoware/planning_topic_converter/converter_base.hpp` |
| `output_topic` | `sources/autoware_workspace/core/autoware_core/planning/autoware_planning_topic_converter/include/autoware/planning_topic_converter/converter_base.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| _none detected_ |  |  |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_utils_geometry`, `rclcpp`, `rclcpp_components`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 12 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 0 |
| `pluginlib` | 0 |
| `parameters` | 2 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/autoware_planning_topic_converter/README.md`
