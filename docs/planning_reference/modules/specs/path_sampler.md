> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_path_sampler

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler`
- Version: `0.51.0`
- Package description: Package for the sampling-based path planner
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::path_sampler::PathSampler` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |

## Purpose And Role

This package implements a node that uses sampling based planning to generate a drivable trajectory. This package is able to: - make the trajectory smooth; - keep the trajectory inside the drivable area; - avoid static obstacles; - stop if no valid trajectory can be generated. Note that the velocity is just taken over from the input path. Sampling based planning is decomposed into 3 successive steps: 1. Sampling: candidate trajectories are generated.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/path` | `Path` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/include/autoware_path_sampler/node.hpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/include/autoware_path_sampler/node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/path` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `~/virtual_wall` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `~/debug/marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `~/debug/calculation_time` | `StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `constraints.hard.max_curvature` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `constraints.hard.min_curvature` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `constraints.hard.min_distance_from_obstacles` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `constraints.hard.limit_footprint_inside_drivable_area` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `constraints.soft.lateral_deviation_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `constraints.soft.length_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `constraints.soft.curvature_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `path_reuse.maximum_lateral_deviation` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `path_reuse.direct_reuse_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.enable_frenet` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.enable_bezier` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.resolution` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.previous_path_reuse_points_nb` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.nb_target_lateral_positions` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.bezier.nb_k` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.bezier.mk_min` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.bezier.mk_max` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.bezier.nb_t` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.bezier.mt_min` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `sampling.bezier.mt_max` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `preprocessing.force_zero_initial_deviation` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `preprocessing.force_zero_initial_heading` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `preprocessing.smooth_reference_trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `debug.enable_calculation_time_info` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `debug.id` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/src/node.cpp` |
| `common.output_delta_arc_length` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/include/autoware_path_sampler/common_structs.hpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/include/autoware_path_sampler/common_structs.hpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/include/autoware_path_sampler/common_structs.hpp` |

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

`ament_cmake_auto`, `ament_cmake_ros`, `ament_index_python`, `ament_lint_auto`, `autoware_bezier_sampler`, `autoware_cmake`, `autoware_fake_test_node`, `autoware_frenet_planner`, `autoware_internal_debug_msgs`, `autoware_interpolation`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_testing`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `std_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 26 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 5 |
| `pluginlib` | 0 |
| `parameters` | 32 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler/README.md`
