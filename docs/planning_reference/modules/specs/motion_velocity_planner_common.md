> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_planner_common

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common`
- Version: `1.8.0`
- Package description: Common functions and interfaces for motion_velocity_planner modules
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| _none detected_ |  |  |

## Purpose And Role

This package provides common utilities and data structures for the motion velocity planner in the Autoware system. It contains tools for geometric calculations, trajectory processing, and velocity planning results. Package motion velocity planner is responsible for generating velocity profiles for autonomous vehicles based on the current trajectory and environment. This package `autoware_motion_velocity_planner_common` contains essential utilities and structures that support the planning process, including:

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
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/planner_data.cpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/planner_data.cpp` |
| `trajectory_polygon_collision_check.decimate_trajectory_step_length` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/planner_data.cpp` |
| `trajectory_polygon_collision_check.goal_extended_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/planner_data.cpp` |
| `trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/planner_data.cpp` |
| `trajectory_polygon_collision_check.consider_current_pose.time_to_convergence` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/src/planner_data.cpp` |
| `<ns_child>enable_monolithic_crop_box` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>enable_multi_polygon_filtering` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>min_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>braking_distance_scale_factor` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>lateral_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>height_margin` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>enable_downsample` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>voxel_size_x` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>voxel_size_y` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>voxel_size_z` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>enable_clustering` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>cluster_tolerance` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>min_cluster_size` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |
| `<ns_child>max_cluster_size` | `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/planner_data.hpp` |

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

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner_common`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils_debug`, `autoware_utils_geometry`, `autoware_utils_math`, `autoware_utils_rclcpp`, `autoware_utils_visualization`, `autoware_velocity_smoother`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `libboost-dev`, `rclcpp`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 53 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 5 |
| `pluginlib` | 0 |
| `parameters` | 22 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/README.md`
