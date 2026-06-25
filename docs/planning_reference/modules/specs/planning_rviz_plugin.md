> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_rviz_plugin

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_rviz_plugins`
- Package path: `core/autoware_rviz_plugins/autoware_planning_rviz_plugin`
- Version: `0.5.0`
- Package description: The autoware_planning_rviz_plugin package
- Primary README: `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `rviz_plugins::AutowarePathWithLaneIdDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/path/display.cpp` |
| pluginlib_plugin | `rviz_plugins::AutowarePathDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/path/display.cpp` |
| pluginlib_plugin | `rviz_plugins::AutowareTrajectoryDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/path/display.cpp` |
| pluginlib_plugin | `rviz_plugins::AutowarePoseWithUuidStampedDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/pose_with_uuid_stamped/display.cpp` |
| pluginlib_plugin | `rviz_plugins::MissionCheckpointTool -> rviz_common::Tool` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/mission_checkpoint/mission_checkpoint.cpp` |
| pluginlib_plugin | `rviz_plugins::AutowareScoredCandidateTrajectoriesDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/candidate_trajectories/scored_candidate_trajectories_display.cpp` |
| pluginlib_plugin | `rviz_plugins::AutowareCandidateTrajectoriesDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/candidate_trajectories/candidate_trajectories_display.cpp` |
| pluginlib_plugin | `rviz_plugins::MaxVelocityDisplay -> rviz_common::Display` | `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/src/tools/max_velocity.cpp` |

## Purpose And Role

This plugin displays the path, trajectory, and maximum speed. 1. Start rviz and select Add under the Displays panel. 2. Select any one of the autoware_planning_rviz_plugin and press OK. 3. Enter the name of the topic where you want to view the path or trajectory. This project uses [Material Design Icons](https://developers.google.com/fonts/docs/material_symbols) by Google. These icons are used under the terms of the Apache License, Version 2.0. Material Design Icons are a collection of symbols provided by Google that are used to enhance the user interface of applications, websites, and other digital products.

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
| _none detected_ |  |  |

## Dependencies

`ament_cmake`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_planning_msgs`, `autoware_utils`, `autoware_vehicle_info_utils`, `libqt5-core`, `libqt5-gui`, `libqt5-widgets`, `qtbase5-dev`, `rclcpp`, `rviz_2d_overlay_msgs`, `rviz_2d_overlay_plugins`, `rviz_common`, `rviz_default_plugins`, `rviz_rendering`, `tf2_geometry_msgs`, `tf2_ros`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 23 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 7 |
| `pluginlib` | 14 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_rviz_plugins/autoware_planning_rviz_plugin/README.md`
