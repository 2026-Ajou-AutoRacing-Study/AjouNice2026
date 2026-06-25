> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_scenario_selector

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_scenario_selector`
- Version: `0.51.0`
- Package description: The autoware_scenario_selector ROS 2 package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::scenario_selector::ScenarioSelectorNode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |

## Purpose And Role

`scenario_selector_node` is a node that switches trajectories from each scenario. None 1. Write your remapping info in `scenario_selector.launch` or add args when executing `roslaunch` 2. `roslaunch autoware_scenario_selector scenario_selector.launch` - If you would like to use only a single scenario, `roslaunch autoware_scenario_selector dummy_scenario_selector_{scenario_name}.launch` {{ json_to_markdown("planning/autoware_scenario_selector/schema/scenario_selector.schema.json") }}

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `input/lane_driving/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `input/parking/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `input/lanelet_map` | `autoware_map_msgs::msg::LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `input/route` | `autoware_planning_msgs::msg::LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `output/scenario` | `autoware_internal_planning_msgs::msg::Scenario` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `update_rate` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `th_max_message_delay_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `th_arrived_distance_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `th_stopped_time_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `th_stopped_velocity_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |
| `enable_mode_switching` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/src/node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `update_rate` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/config/scenario_selector.param.yaml` |
| `th_max_message_delay_sec` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/config/scenario_selector.param.yaml` |
| `th_arrived_distance_m` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/config/scenario_selector.param.yaml` |
| `th_stopped_time_sec` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/config/scenario_selector.param.yaml` |
| `th_stopped_velocity_mps` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/config/scenario_selector.param.yaml` |
| `enable_mode_switching` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/config/scenario_selector.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_agnocast_wrapper`, `autoware_cmake`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_route_handler`, `autoware_utils`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `ros2cli`, `tf2`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `topic_tools`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 36 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 4 |
| `pluginlib` | 0 |
| `parameters` | 6 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_scenario_selector/README.md`
