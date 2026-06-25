> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_external_velocity_limit_selector

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_external_velocity_limit_selector`
- Version: `0.51.0`
- Package description: The autoware_external_velocity_limit_selector ROS 2 package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::external_velocity_limit_selector::ExternalVelocityLimitSelectorNode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp` |

## Purpose And Role

The `external_velocity_limit_selector_node` is a node that keeps consistency of external velocity limits. This module subscribes 1. velocity limit command sent by **API**, 2. velocity limit command sent by **Autoware internal modules**. VelocityLimit.msg contains not only **max velocity** but also information about the **acceleration/jerk constraints** on deceleration. The `external_velocity_limit_selector_node` integrates the lowest velocity limit and the highest jerk constraint to calculate the **hardest velocity limit** that protects all the deceleration points and max velocities sent by API and Autoware internal modules.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `input/velocity_limit_from_api` | `VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp` |
| `input/velocity_limit_from_internal` | `VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp` |
| `input/velocity_limit_clear_command_from_internal` | `VelocityLimitClearCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `output/external_velocity_limit` | `VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp` |
| `output/debug` | `StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp` |

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
| `max_velocity` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default.param.yaml` |
| `max_vel` | `11.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `normal.min_acc` | `-0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `normal.max_acc` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `normal.min_jerk` | `-0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `normal.max_jerk` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `limit.min_acc` | `-2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `limit.max_acc` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `limit.min_jerk` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |
| `limit.max_jerk` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/config/default_common.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `generate_parameter_library`, `rclcpp`, `rclcpp_components`, `ros2cli`, `ros_testing`, `topic_tools`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 14 |
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

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_external_velocity_limit_selector/README.md`
