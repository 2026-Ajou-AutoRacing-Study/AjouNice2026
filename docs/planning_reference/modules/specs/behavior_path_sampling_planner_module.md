> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_sampling_planner_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_sampling_planner_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::SamplingPlannerModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/src/manager.cpp` |

## Purpose And Role

WARNING: This module is experimental and has not been properly tested on a real vehicle, use only on simulations. This package implements a node that uses sampling based planning to generate a drivable trajectory for the behavior path planner. It is heavily based off the [sampling_based_planner module](https://github.com/autowarefoundation/autoware_universe/tree/main/planning/sampling_based_planner). This package is able to: - create a smooth trajectory to avoid static obstacles.

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
| `common.output_delta_arc_length` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `debug.enable_calculation_time_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `debug.id` | `0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `preprocessing.force_zero_initial_deviation` | `True` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `preprocessing.force_zero_initial_heading` | `True` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `preprocessing.smooth_reference_trajectory` | `False` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `constraints.hard.max_curvature` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `constraints.hard.min_curvature` | `-3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `constraints.soft.lateral_deviation_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `constraints.soft.length_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `constraints.soft.curvature_weight` | `2000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `constraints.soft.weights` | `[0.5,1.0,20.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.enable_frenet` | `True` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.enable_bezier` | `False` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.resolution` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.previous_path_reuse_points_nb` | `2` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.target_lengths` | `[20.0, 40.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.nb_target_lateral_positions` | `0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.target_lateral_positions` | `[-4.5,-2.5, 0.0, 2.5,4.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.frenet.target_lateral_velocities` | `[-0.5, 0.0, 0.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |
| `sampling.frenet.target_lateral_accelerations` | `[0.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/config/sampling_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_planner_common`, `autoware_bezier_sampler`, `autoware_cmake`, `autoware_frenet_planner`, `autoware_interpolation`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_path_sampler`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_route_handler`, `autoware_signal_processing`, `autoware_utils`, `autoware_vehicle_info_utils`, `autoware_vehicle_msgs`, `geometry_msgs`, `pluginlib`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 21 |
| `lifecycle` | 0 |
| `component` | 1 |
| `tf2` | 6 |
| `pluginlib` | 3 |
| `parameters` | 18 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module/README.md`
