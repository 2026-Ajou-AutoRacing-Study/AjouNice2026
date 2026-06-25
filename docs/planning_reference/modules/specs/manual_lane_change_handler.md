> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_manual_lane_change_handler

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_manual_lane_change_handler`
- Version: `0.51.0`
- Package description: Manual lane change handler node extracted from autoware_mission_planner_universe
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::manual_lane_change_handler::ManualLaneChangeHandler` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |

## Purpose And Role

`Manual Lane Change Handler` helps shift the preferred-primitives of a route to the left or right. A manual lane-change handler rviz plugin is also available for use for the user. Service call to shift the preferred-lanes to the left This service allows for shifting the current preferred lane to the left or right, or reverting to the default preferred lane received from the route planner. As the vehicle moves, the shifting is done relative to the current preferred lane.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |
| `~/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |
| `/planning/mission_planning/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |
| `~/input/reroute_availability` | `tier4_planning_msgs::msg::RerouteAvailability` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |
| `~/debug/shift_number` | `autoware_internal_debug_msgs::msg::Int32Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `~/set_preferred_lane` | `SetPreferredLane` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |
| `/planning/mission_planning/mission_planner/set_preferred_primitive` | `SetPreferredPrimitive` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/src/manual_lane_change_handler.cpp` |

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

`ament_cmake_auto`, `autoware_internal_debug_msgs`, `autoware_lanelet2_extension`, `autoware_map_msgs`, `autoware_mission_planner_universe`, `autoware_planning_msgs`, `autoware_route_handler`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `tier4_external_api_msgs`, `tier4_planning_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 19 |
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

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_manual_lane_change_handler/README.md`
