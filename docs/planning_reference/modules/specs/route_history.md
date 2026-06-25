> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_route_history

## Identity

- Source set: `release_workspace`
- Repository: `tools`
- Package path: `tools/planning/autoware_route_history`
- Version: `0.7.0`
- Package description: Custom RViz panel for the save and replay of recorded routes between poses in the Autoware planning tool.
- Primary README: `sources/autoware_workspace/tools/planning/autoware_route_history/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::route_history::RouteHistoryPanel -> rviz_common::Panel` | `sources/autoware_workspace/tools/planning/autoware_route_history/src/rviz_panel.cpp` |

## Purpose And Role

This package provides an interactive RViz panel for saving poses, enabling the quick (re)generation of routes for simulation and testing. The repeatable testing of a specific route is slowed by the need to replot. This may be done manually through "Add goal pose" and similar toolbar tools, but use of the GUI is prone to inaccuracy. Alternatively, if poses are recorded separately, service calls may uses. This RViz panel aims to streamline this process.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `/api/routing/route` | `adapi_route` | `sources/autoware_workspace/tools/planning/autoware_route_history/include/route_history/node_logic.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | `sources/autoware_workspace/tools/planning/autoware_route_history/include/route_history/node_logic.hpp` |
| `/planning/mission_planning/goal` | `geometry_msgs::msg::PoseStamped` | `sources/autoware_workspace/tools/planning/autoware_route_history/include/route_history/node_logic.hpp` |
| `update` | `std_msgs::msg::String` | `sources/autoware_workspace/tools/planning/autoware_route_history/include/route_history/node_logic.hpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `save_file_path` | `sources/autoware_workspace/tools/planning/autoware_route_history/include/route_history/node_logic.hpp` |

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

`ament_cmake`, `ament_lint_auto`, `ament_lint_common`, `autoware_adapi_v1_msgs`, `geometry_msgs`, `pluginlib`, `qtbase5-dev`, `rclcpp`, `rviz2`, `rviz_common`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 18 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 3 |
| `parameters` | 2 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/tools/planning/autoware_route_history/README.md`
