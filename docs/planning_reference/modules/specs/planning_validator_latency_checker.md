> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_validator_latency_checker

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker`
- Version: `0.51.0`
- Package description: The autoware_planning_validator_latency_checker package
- Primary README: `not found`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::planning_validator::LatencyChecker -> autoware::planning_validator::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker/src/latency_checker.cpp` |

## Purpose And Role

No concise README paragraph was detected. Use the source files below as primary evidence.

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
| `latency_checker.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker/src/latency_checker.cpp` |
| `latency_checker.threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker/src/latency_checker.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `latency_checker.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker/config/latency_checker.param.yaml` |
| `latency_checker.threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker/config/latency_checker.param.yaml` |
| `latency_checker.is_critical` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker/config/latency_checker.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_lint_common`, `autoware_planning_validator`, `autoware_trajectory`, `autoware_utils`, `pluginlib`, `rclcpp`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 5 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 3 |
| `parameters` | 3 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- No markdown files found under this package.
