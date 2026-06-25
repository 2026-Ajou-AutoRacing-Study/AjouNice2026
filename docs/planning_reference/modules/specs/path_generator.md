> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_path_generator

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/autoware_path_generator`
- Version: `1.8.0`
- Package description: The autoware_path_generator package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::path_generator::PathGenerator` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/src/node.cpp` |

## Purpose And Role

The `path_generator` node receives a route from `mission_planner` and converts the center line into a path. If the route has waypoints set, it generates a path passing through them. This package is a simple alternative of `behavior_path_generator`. When input data is ready, it first searches for the lanelet closest to the vehicle. If found, it gets the lanelets within a distance of `path_length.backward` behind and `path_length.forward` in front. Their center lines are concatenated to generate a path.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/route` | `LaneletRoute, autoware_utils_rclcpp::polling_policy::Newest` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/include/autoware/path_generator/node.hpp` |
| `~/input/vector_map` | `LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/include/autoware/path_generator/node.hpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/include/autoware/path_generator/node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/path` | `PathWithLaneId` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/src/node.cpp` |
| `~/output/turn_indicators_cmd` | `TurnIndicatorsCommand` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/src/node.cpp` |
| `~/output/hazard_lights_cmd` | `HazardLightsCommand` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/src/node.cpp` |
| `~/debug/processing_time_detail_ms` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/src/node.cpp` |
| `~/debug/processing_time_ms` | `Float64Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/src/node.cpp` |

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
| test | parameter | `path_length.backward` | `` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/test/test_dense_centerline.cpp` |
| test | parameter | `path_length.forward` | `` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/test/test_dense_centerline.cpp` |
| test | parameter | `waypoint.connection_gradient_from_centerline` | `` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/test/test_dense_centerline.cpp` |
| test | parameter | `goal_connection.connection_section_length` | `` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/test/test_dense_centerline.cpp` |
| test | parameter | `smooth_goal_connection.pre_goal_offset` | `` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/test/test_dense_centerline.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `planning_hz` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `path_length.backward` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `path_length.forward` | `300.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `waypoint.connection_gradient_from_centerline` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `turn_signal.search_time` | `3.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `turn_signal.search_distance` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `turn_signal.resampling_interval` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `turn_signal.angle_threshold_deg` | `15.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `goal_connection.connection_section_length` | `7.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |
| `goal_connection.pre_goal_offset` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/config/path_generator.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_test_utils`, `autoware_trajectory`, `autoware_utils_debug`, `autoware_utils_rclcpp`, `autoware_utils_system`, `autoware_vehicle_info_utils`, `generate_parameter_library`, `rclcpp`, `rclcpp_components`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 42 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 2 |
| `pluginlib` | 0 |
| `parameters` | 5 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/autoware_path_generator/README.md`
