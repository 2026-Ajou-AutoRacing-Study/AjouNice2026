> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_path_smoother

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_path_smoother`
- Version: `0.51.0`
- Package description: The autoware_path_smoother package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::path_smoother::ElasticBandSmoother` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |

## Purpose And Role

This package contains code to smooth a path or trajectory. More details about the elastic band can be found here. {{ json_to_markdown("planning/autoware_path_smoother/schema/elastic_band_smoother.schema.json") }}

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/path` | `Path` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/include/autoware/path_smoother/elastic_band_smoother.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/eb_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `~/debug/eb_fixed_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `~/output/traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |
| `~/output/path` | `Path` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |
| `~/debug/extended_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |
| `~/debug/calculation_time` | `StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |
| `~/debug/processing_time_ms` | `Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band_smoother.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `replan.enable` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `replan.max_path_shape_around_ego_lat_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `replan.max_path_shape_forward_lat_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `replan.max_path_shape_forward_lon_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `replan.max_ego_moving_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `replan.max_goal_moving_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `replan.max_delta_time_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/replan_checker.cpp` |
| `elastic_band.option.enable_warm_start` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.option.enable_optimization_validation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.common.delta_arc_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.common.num_points` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.clearance.num_joint_points` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.clearance.clearance_for_fix` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.clearance.clearance_for_joint` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.clearance.clearance_for_smooth` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.weight.smooth_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.weight.lat_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.qp.max_iteration` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.qp.eps_abs` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.qp.eps_rel` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `elastic_band.validation.max_error` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/src/elastic_band.cpp` |
| `common.output_backward_traj_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/include/autoware/path_smoother/common_structs.hpp` |
| `common.output_delta_arc_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/include/autoware/path_smoother/common_structs.hpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/include/autoware/path_smoother/common_structs.hpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/include/autoware/path_smoother/common_structs.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `common.output_delta_arc_length` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `common.output_backward_traj_length` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.option.enable_warm_start` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.option.enable_optimization_validation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.common.num_points` | `100` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.common.delta_arc_length` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.num_joint_points` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.clearance_for_fix` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.clearance_for_joint` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.clearance_for_smooth` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.weight.smooth_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.weight.lat_error_weight` | `0.001` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.qp.max_iteration` | `10000` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.qp.eps_abs` | `1.0e-7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.qp.eps_rel` | `1.0e-7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `elastic_band.validation.max_error` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `ego_nearest_dist_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `ego_nearest_yaw_threshold` | `1.046` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.max_path_shape_around_ego_lat_dist` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.max_path_shape_forward_lon_dist` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.max_path_shape_forward_lat_dist` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.max_ego_moving_dist` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.max_goal_moving_dist` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |
| `replan.max_delta_time_sec` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_index_python`, `ament_lint_auto`, `autoware_cmake`, `autoware_fake_test_node`, `autoware_internal_debug_msgs`, `autoware_interpolation`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_osqp_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_testing`, `autoware_utils`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `std_msgs`, `tf2_ros`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 56 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 3 |
| `pluginlib` | 0 |
| `parameters` | 25 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/README.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_smoother/docs/eb.md`
