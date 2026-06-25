> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_velocity_smoother

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/autoware_velocity_smoother`
- Version: `1.8.0`
- Package description: The autoware_velocity_smoother package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::velocity_smoother::VelocitySmootherNode` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |

## Purpose And Role

`autoware_velocity_smoother` outputs a desired velocity profile on a reference trajectory. This module plans a velocity profile within the limitations of the velocity, the acceleration and the jerk to realize both the maximization of velocity and the ride quality. We call this module `autoware_velocity_smoother` because the limitations of the acceleration and the jerk means the smoothness of the velocity profile. For the point on the reference trajectory closest to the center of the rear wheel axle of the vehicle, it extracts the reference path between `extract_behind_dist` behind and `extract_ahead_dist` ahead.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/trajectory` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `/localization/kinematic_state` | `Odometry` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/include/autoware/velocity_smoother/node.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/include/autoware/velocity_smoother/node.hpp` |
| `~/input/external_velocity_limit_mps` | `VelocityLimit, autoware_utils_rclcpp::polling_policy::Newest` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/include/autoware/velocity_smoother/node.hpp` |
| `~/input/operation_mode_state` | `OperationModeState` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/include/autoware/velocity_smoother/node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/processing_time_detail_ms` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/output/trajectory` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/virtual_wall` | `MarkerArray` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/output/current_velocity_limit_mps` | `VelocityLimit` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/distance_to_stopline` | `Float32Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/closest_velocity` | `Float32Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/closest_acceleration` | `Float32Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/closest_jerk` | `Float32Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/closest_max_velocity` | `Float32Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/processing_time_ms` | `Float64Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/trajectory_raw` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/trajectory_external_velocity_limited` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/trajectory_lateral_acc_filtered` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/trajectory_steering_rate_limited` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/trajectory_time_resampled` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/forward_filtered_trajectory` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/backward_filtered_trajectory` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/debug/merged_filtered_trajectory` | `Trajectory` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `~/closest_merged_velocity` | `Float32Stamped` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `over_stop_velocity_warn_thr` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `publish_debug_trajs` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `max_vel` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `margin_to_insert_external_velocity_limit` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `replan_vel_deviation` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `engage_velocity` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `engage_acceleration` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `engage_exit_ratio` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `stopping_velocity` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `stopping_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `extract_ahead_dist` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `extract_behind_dist` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `stop_dist_to_prohibit_engage` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `normal.max_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `normal.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `stop_decel` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `normal.max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `normal.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `min_curve_velocity` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `decel_distance_before_curve` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `decel_distance_after_curve` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `max_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `min_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `resample_time` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `dense_resample_dt` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `min_interval_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `sparse_resample_dt` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `sparse_min_interval_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `resample_ds` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `curvature_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `curvature_calculation_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `jerk_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `over_v_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `over_a_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `over_j_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `jerk_filter_ds` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `pseudo_jerk_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `resample.delta_yaw_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `latacc.constant_velocity_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `forward.max_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `forward.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `forward.max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `forward.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `forward.kp` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `backward.start_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `backward.min_jerk_mild_stop` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `backward.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `backward.min_acc_mild_stop` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `backward.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `backward.span_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `enable_lateral_acc_limit` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `enable_steering_rate_limit` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_max_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_min_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_resample_time` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_dense_resample_dt` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_dense_min_interval_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_sparse_resample_dt` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `post_sparse_min_interval_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `algorithm_type` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `plan_from_ego_speed_on_manual_mode` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/node.cpp` |
| `pseudo_jerk_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/linf_pseudo_jerk_smoother.cpp` |
| `over_v_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/linf_pseudo_jerk_smoother.cpp` |
| `over_a_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/linf_pseudo_jerk_smoother.cpp` |
| `jerk_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp` |
| `over_v_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp` |
| `over_a_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp` |
| `over_j_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp` |
| `jerk_filter_ds` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp` |
| `pseudo_jerk_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/l2_pseudo_jerk_smoother.cpp` |
| `over_v_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/l2_pseudo_jerk_smoother.cpp` |
| `over_a_weight` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/l2_pseudo_jerk_smoother.cpp` |
| `normal.max_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `normal.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `stop_decel` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `normal.max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `normal.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `min_decel_for_lateral_acc_lim_filter` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `resample_ds` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `curvature_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `curvature_calculation_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `decel_distance_before_curve` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `decel_distance_after_curve` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `min_curve_velocity` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `max_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `min_trajectory_length` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `resample_time` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `dense_resample_dt` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `dense_min_interval_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `sparse_resample_dt` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `sparse_min_interval_distance` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/smoother_base.cpp` |
| `resample.ds_resample` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `resample.num_resample` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `resample.delta_yaw_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `latacc.enable_constant_velocity_while_turning` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `latacc.constant_velocity_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `forward.max_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `forward.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `forward.max_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `forward.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `forward.kp` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `backward.start_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `backward.min_jerk_mild_stop` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `backward.min_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `backward.min_acc_mild_stop` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `backward.min_acc` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |
| `backward.span_jerk` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `resample.ds_resample` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `resample.num_resample` | `1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `resample.delta_yaw_threshold` | `0.785` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `latacc.enable_constant_velocity_while_turning` | `false` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `latacc.constant_velocity_dist_threshold` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `forward.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `forward.min_acc` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `forward.max_jerk` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `forward.min_jerk` | `-0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `forward.kp` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `backward.start_jerk` | `-0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `backward.min_jerk_mild_stop` | `-0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `backward.min_jerk` | `-1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `backward.min_acc_mild_stop` | `-1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `backward.min_acc` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `backward.span_jerk` | `-0.01` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Analytical.param.yaml` |
| `jerk_weight` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/JerkFiltered.param.yaml` |
| `over_v_weight` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/JerkFiltered.param.yaml` |
| `over_a_weight` | `5000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/JerkFiltered.param.yaml` |
| `over_j_weight` | `2000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/JerkFiltered.param.yaml` |
| `jerk_filter_ds` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/JerkFiltered.param.yaml` |
| `pseudo_jerk_weight` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Linf.param.yaml` |
| `over_v_weight` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Linf.param.yaml` |
| `over_a_weight` | `5000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/Linf.param.yaml` |
| `max_velocity` | `20.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `stop_decel` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `margin_to_insert_external_velocity_limit` | `0.3` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `curvature_calculation_distance` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `enable_lateral_acc_limit` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `lateral_acceleration_limits` | `[0.8, 0.8, 0.8, 0.8]` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `min_curve_velocity` | `2.74` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `decel_distance_before_curve` | `3.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `decel_distance_after_curve` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `min_decel_for_lateral_acc_lim_filter` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `enable_steering_rate_limit` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `velocity_thresholds` | `[0.1, 0.3, 20.0, 30.0]` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `steering_angle_rate_limits` | `[57.0, 57.0, 57.0, 40.0]` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `resample_ds` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `curvature_threshold` | `0.02` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `replan_vel_deviation` | `5.53` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `engage_velocity` | `0.25` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `engage_acceleration` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `engage_exit_ratio` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `stop_dist_to_prohibit_engage` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `stopping_velocity` | `2.778` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `stopping_distance` | `0.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `extract_ahead_dist` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `extract_behind_dist` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `delta_yaw_threshold` | `1.0472` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `max_trajectory_length` | `200.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `min_trajectory_length` | `150.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `resample_time` | `2.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `dense_resample_dt` | `0.2` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `dense_min_interval_distance` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `sparse_resample_dt` | `0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `sparse_min_interval_distance` | `4.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_max_trajectory_length` | `300.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_min_trajectory_length` | `30.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_resample_time` | `10.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_dense_resample_dt` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_dense_min_interval_distance` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_sparse_resample_dt` | `0.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `post_sparse_min_interval_distance` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `over_stop_velocity_warn_thr` | `1.389` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `plan_from_ego_speed_on_manual_mode` | `true` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml` |
| `max_vel` | `11.1` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `normal.min_acc` | `-0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `normal.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `normal.min_jerk` | `-0.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `normal.max_jerk` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `limit.min_acc` | `-2.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `limit.max_acc` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `limit.min_jerk` | `-1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `limit.max_jerk` | `1.5` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/default_common.param.yaml` |
| `pseudo_jerk_weight` | `100.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/L2.param.yaml` |
| `over_v_weight` | `100000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/L2.param.yaml` |
| `over_a_weight` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/config/L2.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_osqp_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_qp_interface`, `autoware_trajectory`, `autoware_utils_debug`, `autoware_utils_diagnostics`, `autoware_utils_geometry`, `autoware_utils_logging`, `autoware_utils_math`, `autoware_utils_rclcpp`, `autoware_vehicle_info_utils`, `eigen3_cmake_module`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `tf2`, `tf2_ros`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 96 |
| `lifecycle` | 0 |
| `component` | 2 |
| `tf2` | 20 |
| `pluginlib` | 0 |
| `parameters` | 78 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/autoware_velocity_smoother/README.md`
