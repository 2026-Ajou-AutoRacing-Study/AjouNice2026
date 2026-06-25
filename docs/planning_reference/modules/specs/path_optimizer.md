> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_path_optimizer

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_path_optimizer`
- Version: `0.51.0`
- Package description: The autoware_path_optimizer package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::path_optimizer::PathOptimizer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |

## Purpose And Role

This package generates a trajectory that is kinematically-feasible to drive and collision-free based on the input path, drivable area. Only position and orientation of trajectory are updated in this module, and velocity is just taken over from the one in the input path. This package is able to - make the trajectory inside the drivable area as much as possible - NOTE: Static obstacles to avoid can be removed from the drivable area. - insert stop point before the planned footprint will be outside the drivable area

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/path` | `Path` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/include/autoware/path_optimizer/node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/path` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/virtual_wall` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/debug/extended_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/debug/marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/debug/calculation_time` | `StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/debug/processing_time_ms` | `Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/debug/processing_time_detail_ms` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `~/debug/mpt_fixed_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/mpt_ref_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/mpt_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/optimised_steering` | `std_msgs::msg::Float32MultiArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/acados_mpt_traj` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/acados_optimised_steering` | `std_msgs::msg::Float32MultiArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/optimised_states` | `std_msgs::msg::Float32MultiArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/acados_optimised_states` | `std_msgs::msg::Float32MultiArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `~/debug/ref_steering` | `std_msgs::msg::Float32MultiArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `replan.max_path_shape_around_ego_lat_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/replan_checker.cpp` |
| `replan.max_path_shape_forward_lat_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/replan_checker.cpp` |
| `replan.max_path_shape_forward_lon_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/replan_checker.cpp` |
| `replan.max_ego_moving_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/replan_checker.cpp` |
| `replan.max_goal_moving_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/replan_checker.cpp` |
| `replan.max_delta_time_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/replan_checker.cpp` |
| `option.enable_outside_drivable_area_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `option.enable_skip_optimization` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `option.enable_reset_prev_optimization` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `option.use_footprint_polygon_for_outside_drivable_area_check` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `option.debug.enable_pub_debug_marker` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `option.debug.enable_pub_extra_debug_marker` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `option.debug.enable_debug_info` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `common.vehicle_stop_margin_outside_drivable_area` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/node.cpp` |
| `mpt.option.steer_limit_constraint` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.option.enable_warm_start` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.option.enable_manual_warm_start` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.option.enable_optimization_validation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.option.visualize_sampling_num` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.common.num_points` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.common.delta_arc_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.use_acados` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.use_acados_circle_constraints` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.kinematics.optimization_center_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.clearance.hard_clearance_from_road` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.clearance.soft_clearance_from_road` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.soft_collision_free_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.lat_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.yaw_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.yaw_error_rate_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.steer_input_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.steer_rate_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.terminal_lat_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.terminal_yaw_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.goal_lat_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.weight.goal_yaw_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.max_longitudinal_margin_for_bound_violation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.max_bound_fixing_time` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.max_avoidance_cost` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.avoidance_cost_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.avoidance_cost_band_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.avoidance_cost_decrease_rate` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.min_drivable_width` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.weight.lat_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.weight.yaw_error_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.avoidance.weight.steer_input_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.option.l_inf_norm` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.option.soft_constraint` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.option.hard_constraint` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.vehicle_circles.method` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.vehicle_circles.uniform_circle.num` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.vehicle_circles.` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.validation.max_lat_error` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `mpt.validation.max_yaw_error` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/src/mpt_optimizer.cpp` |
| `common.output_backward_traj_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/include/autoware/path_optimizer/common_structs.hpp` |
| `common.output_delta_arc_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/include/autoware/path_optimizer/common_structs.hpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/include/autoware/path_optimizer/common_structs.hpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/include/autoware/path_optimizer/common_structs.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `option.enable_skip_optimization` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.enable_reset_prev_optimization` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.enable_outside_drivable_area_stop` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.use_footprint_polygon_for_outside_drivable_area_check` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.debug.enable_pub_debug_marker` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.debug.enable_pub_extra_debug_marker` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.debug.enable_debug_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `option.debug.enable_calculation_time_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `common.output_delta_arc_length` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `common.output_backward_traj_length` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `common.vehicle_stop_margin_outside_drivable_area` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `replan.max_path_shape_around_ego_lat_dist` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `replan.max_path_shape_forward_lon_dist` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `replan.max_path_shape_forward_lat_dist` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `replan.max_ego_moving_dist` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `replan.max_goal_moving_dist` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `replan.max_delta_time_sec` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.use_acados` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.use_acados_circle_constraints` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.option.steer_limit_constraint` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.option.visualize_sampling_num` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.option.enable_manual_warm_start` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.option.enable_warm_start` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.option.enable_optimization_validation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.common.num_points` | `100` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.common.delta_arc_length` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.clearance.hard_clearance_from_road` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.clearance.soft_clearance_from_road` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.soft_collision_free_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.lat_error_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.yaw_error_weight` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.yaw_error_rate_weight` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.steer_input_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.steer_rate_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.terminal_lat_error_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.terminal_yaw_error_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.goal_lat_error_weight` | `1000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.weight.goal_yaw_error_weight` | `1000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.max_bound_fixing_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.max_longitudinal_margin_for_bound_violation` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.max_avoidance_cost` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.avoidance_cost_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.avoidance_cost_band_length` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.avoidance_cost_decrease_rate` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.min_drivable_width` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.weight.lat_error_weight` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.weight.yaw_error_weight` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.avoidance.weight.steer_input_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.option.l_inf_norm` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.option.soft_constraint` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.option.hard_constraint` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.method` | `"fitting_uniform_circle"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_calculation` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.front_radius_ratio` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.rear_radius_ratio` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.uniform_circle.num` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.validation.max_lat_error` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |
| `mpt.validation.max_yaw_error` | `1.046` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/config/path_optimizer.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_export_link_flags`, `ament_cmake_ros`, `ament_index_python`, `ament_lint_auto`, `autoware_cmake`, `autoware_fake_test_node`, `autoware_global_parameter_loader`, `autoware_internal_debug_msgs`, `autoware_interpolation`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_osqp_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_testing`, `autoware_utils`, `autoware_utils_geometry`, `autoware_vehicle_info_utils`, `diagnostic_updater`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `std_msgs`, `tf2_ros`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 86 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 11 |
| `pluginlib` | 0 |
| `parameters` | 65 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/README.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/docs/debug.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_path_optimizer/docs/mpt.md`
