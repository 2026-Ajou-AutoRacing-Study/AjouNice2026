> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_hazard_lights_selector

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_hazard_lights_selector`
- Version: `0.51.0`
- Package description: The autoware_hazard_lights_selector package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::hazard_lights_selector::HazardLightsSelector` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/src/node.cpp` |

## Purpose And Role

The Hazard Lights Selector is a node that manages and selects hazard light commands from multiple sources (planning and system) to determine the final hazard lights state of the vehicle. The node operates on a timer-based update cycle and implements the following logic: 1. Receives hazard light commands from both planning and system sources 2. If either source requests hazard lights to be enabled, the final command will be to enable hazard lights 3. If neither source requests hazard lights to be enabled, the final command will be to disable hazard lights

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `input/planning/hazard_lights_command` | `autoware_vehicle_msgs::msg::HazardLightsCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/src/node.cpp` |
| `input/system/hazard_lights_command` | `autoware_vehicle_msgs::msg::HazardLightsCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/src/node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `output/hazard_lights_command` | `autoware_vehicle_msgs::msg::HazardLightsCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/src/node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `update_rate` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/src/node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `update_rate` | `10` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/config/hazard_lights_selector.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_cmake`, `autoware_lint_common`, `autoware_vehicle_msgs`, `rclcpp`, `rclcpp_components`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 11 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 0 |
| `pluginlib` | 0 |
| `parameters` | 1 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_hazard_lights_selector/README.md`
