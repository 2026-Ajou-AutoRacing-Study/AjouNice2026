> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_planner_common

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common`
- Version: `1.8.0`
- Package description: The autoware_behavior_velocity_planner_common package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| _none detected_ |  |  |

## Purpose And Role

This package provides a behavior velocity interface without RTC, and common functions as a library, which are used in the `behavior_velocity_planner` node and modules.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/processing_time_detail_ms/` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp` |
| `~/debug/processing_time_detail_ms/` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `max_accel` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/planner_data.cpp` |
| `max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/planner_data.cpp` |
| `system_delay` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/planner_data.cpp` |
| `delay_response_time` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/src/planner_data.cpp` |
| `is_publish_debug_path` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp` |
| `planning_factor_console_output.enable` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp` |
| `planning_factor_console_output.duration` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp` |
| `is_publish_debug_path` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp` |
| `planning_factor_console_output.enable` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp` |
| `planning_factor_console_output.duration` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `max_accel` | `-2.8` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/config/behavior_velocity_planner_common.param.yaml` |
| `max_jerk` | `-5.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/config/behavior_velocity_planner_common.param.yaml` |
| `system_delay` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/config/behavior_velocity_planner_common.param.yaml` |
| `delay_response_time` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/config/behavior_velocity_planner_common.param.yaml` |
| `is_publish_debug_path` | `false` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/config/behavior_velocity_planner_common.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_adapi_v1_msgs`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_objects_of_interest_marker_interface`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_test_utils`, `autoware_trajectory`, `autoware_utils_debug`, `autoware_utils_geometry`, `autoware_utils_rclcpp`, `autoware_utils_system`, `autoware_utils_visualization`, `autoware_vehicle_info_utils`, `autoware_velocity_smoother`, `diagnostic_msgs`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `nav_msgs`, `pcl_conversions`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 93 |
| `lifecycle` | 0 |
| `component` | 1 |
| `tf2` | 20 |
| `pluginlib` | 0 |
| `parameters` | 14 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/README.md`
