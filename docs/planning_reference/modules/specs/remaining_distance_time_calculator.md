> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_remaining_distance_time_calculator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_remaining_distance_time_calculator`
- Version: `0.51.0`
- Package description: Calculates and publishes remaining distance and time of the mission.
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |

## Purpose And Role

This package aims to provide mission remaining distance and remaining time calculations. - The calculations are activated once we have a route planned for a mission in Autoware. - The calculations are triggered timely based on the `update_rate` parameter. - The calculations are skipped if the scenario is PARKING, and the remaining time and distance values are set to 0.0. - The remaining distance calculation is based on getting the remaining shortest path between the current vehicle pose and goal pose using `lanelet2` routing APIs.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |
| `~/input/map` | `HADMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |
| `~/input/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |
| `/planning/scenario_planning/current_max_velocity` | `autoware_internal_planning_msgs::msg::VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |
| `~/input/scenario` | `autoware_internal_planning_msgs::msg::Scenario` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/mission_remaining_distance_time` | `MissionRemainingDistanceTime` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |
| `~/debug/processing_time_detail_ms` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/src/remaining_distance_time_calculator_node.cpp` |

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
| `update_rate` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/config/remaining_distance_time_calculator.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_msgs`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_test_utils`, `autoware_utils`, `autoware_utils_debug`, `generate_parameter_library`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `std_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 22 |
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

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_remaining_distance_time_calculator/README.md`
