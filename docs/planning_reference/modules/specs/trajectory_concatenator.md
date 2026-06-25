> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_trajectory_concatenator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_trajectory_concatenator`
- Version: `0.51.0`
- Package description: The autoware_trajectory_concatenator package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::trajectory_concatenator::TrajectoryConcatenatorNode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/src/trajectory_concatenator_node.cpp` |

## Purpose And Role

This node aggregates trajectory candidates from multiple trajectory generators into a single `autoware_internal_planning_msgs/msg/CandidateTrajectories` message. It is intended to be placed between trajectory generators and the selector/ranker. When a message arrives on `~/input/trajectories`, the node splits it by `generator_id` and updates an in-memory buffer so that only the most recent trajectory set for each generator is retained. A 100 ms timer then scans this buffer and drops any entry whose header stamp is older than the configured duration_time. Immediately after pruning, the timer concatenates all remaining trajectories and their accompanying `generator_info` arrays, publishes the 

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/src/trajectory_concatenator_node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/src/trajectory_concatenator_node.cpp` |
| `~/debug/processing_time_detail_ms/trajectory_concatenator` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/src/trajectory_concatenator_node.cpp` |

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
| `duration_time` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/config/concatenator.param.yaml` |

## Dependencies

`ament_cmake`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_planning_msgs`, `autoware_utils_debug`, `autoware_utils_rclcpp`, `autoware_utils_uuid`, `generate_parameter_library`, `rclcpp`, `rclcpp_components`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 16 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 0 |
| `pluginlib` | 0 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_concatenator/README.md`
