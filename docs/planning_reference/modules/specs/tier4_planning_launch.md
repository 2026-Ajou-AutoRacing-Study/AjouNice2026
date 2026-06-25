> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# tier4_planning_launch

## Identity

- Source set: `release_workspace`
- Repository: `launcher/autoware_launch`
- Package path: `launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch`
- Version: `0.51.0`
- Package description: The tier4_planning_launch package
- Primary README: `sources/autoware_workspace/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| _none detected_ |  |  |

## Purpose And Role

Please see `<exec_depend>` in `package.xml`. Note that you should provide parameter paths as `PACKAGE_param_path`. The list of parameter paths you should provide is written at the top of `planning.launch.xml`.

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

`agnocast_components`, `ament_cmake_auto`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_velocity_planner`, `autoware_cmake`, `autoware_costmap_generator`, `autoware_external_cmd_selector`, `autoware_external_velocity_limit_selector`, `autoware_freespace_planner`, `autoware_hazard_lights_selector`, `autoware_lint_common`, `autoware_manual_lane_change_handler`, `autoware_mission_planner_universe`, `autoware_path_optimizer`, `autoware_planning_evaluator`, `autoware_planning_topic_converter`, `autoware_planning_validator`, `autoware_remaining_distance_time_calculator`, `autoware_scenario_selector`, `autoware_surround_obstacle_checker`, `autoware_velocity_smoother`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 0 |
| `lifecycle` | 0 |
| `component` | 13 |
| `tf2` | 0 |
| `pluginlib` | 0 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/README.md`
