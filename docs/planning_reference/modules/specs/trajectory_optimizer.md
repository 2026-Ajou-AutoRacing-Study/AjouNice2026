> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_trajectory_optimizer

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_trajectory_optimizer`
- Version: `0.51.0`
- Package description: The autoware_trajectory_optimizer package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::trajectory_optimizer::TrajectoryOptimizer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryMPTOptimizer -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryPointFixer -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp` |
| pluginlib_plugin | `autoware::trajectory_optimizer::plugin::TrajectoryExtender -> autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_extender.cpp` |

## Purpose And Role

The `autoware_trajectory_optimizer` package generates smooth and feasible trajectories for autonomous vehicles using a plugin-based optimization pipeline. It takes candidate trajectories as input and applies a sequence of optimization plugins to produce smooth, drivable trajectories with proper velocity and acceleration profiles. - **Plugin-based architecture** - Modular optimization pipeline where each step is a separate plugin - **Multiple smoothing methods**:

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/include/autoware/trajectory_optimizer/trajectory_optimizer.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/include/autoware/trajectory_optimizer/trajectory_optimizer.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/processing_time_detail_ms` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `~/output/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `~/output/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `~/output/current_velocity_limit_mps` | `VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `~/debug/mpt_processing_time_detail_ms` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `~/debug/mpt_bounds_markers` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `plugin_names` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_akima_spline_interpolation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_eb_smoother` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_qp_smoother` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_trajectory_point_fixer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_velocity_optimizer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_trajectory_extender` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_kinematic_feasibility_enforcer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `use_mpt_optimizer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer.cpp` |
| `trajectory_spline_smoother.interpolation_resolution_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp` |
| `trajectory_spline_smoother.max_distance_discrepancy_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp` |
| `trajectory_spline_smoother.preserve_input_trajectory_orientation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp` |
| `trajectory_velocity_optimizer.nearest_dist_threshold_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.nearest_yaw_threshold_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.target_pull_out_speed_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.target_pull_out_acc_mps2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.max_lateral_accel_mps2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.min_limited_speed_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `max_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.set_engage_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.limit_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.limit_lateral_acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.smooth_velocities` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.jerk_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.over_v_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.over_a_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.over_j_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.velocity_tracking_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.accel_tracking_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `limit.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `limit.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `limit.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `limit.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp` |
| `trajectory_mpt_optimizer.corridor_width_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.enable_adaptive_width` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.curvature_width_factor` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.velocity_width_factor` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.min_clearance_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.reset_previous_data_each_iteration` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.enable_debug_info` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.output_delta_arc_length_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.output_backward_traj_length_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.ego_nearest_dist_threshold_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_mpt_optimizer.acceleration_moving_average_window` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp` |
| `trajectory_kinematic_feasibility.max_yaw_rate_rad_s` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.cpp` |
| `trajectory_kinematic_feasibility.time_step_s` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.cpp` |
| `trajectory_qp_smoother.weight_smoothness` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.weight_fidelity` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.time_step_s` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.osqp_eps_abs` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.osqp_eps_rel` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.osqp_max_iter` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.osqp_verbose` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.preserve_input_trajectory_orientation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.max_distance_for_orientation_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.use_velocity_based_fidelity` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.velocity_threshold_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.sigmoid_sharpness` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.min_fidelity_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.max_fidelity_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.num_constrained_points_start` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_qp_smoother.num_constrained_points_end` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp` |
| `trajectory_point_fixer.remove_close_points` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp` |
| `trajectory_point_fixer.resample_close_points` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp` |
| `trajectory_point_fixer.min_dist_to_remove_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp` |
| `trajectory_point_fixer.min_dist_to_resample_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp` |
| `trajectory_point_fixer.stop_detection_velocity_threshold_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp` |
| `trajectory_extender.nearest_dist_threshold_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_extender.cpp` |
| `trajectory_extender.nearest_yaw_threshold_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_extender.cpp` |
| `trajectory_extender.backward_trajectory_extension_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/src/trajectory_optimizer_plugins/trajectory_extender.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `use_akima_spline_interpolation` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_eb_smoother` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_qp_smoother` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_trajectory_point_fixer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_velocity_optimizer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_trajectory_extender` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_kinematic_feasibility_enforcer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `use_mpt_optimizer` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.nearest_dist_threshold_m` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.nearest_yaw_threshold_deg` | `60.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.target_pull_out_speed_mps` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.target_pull_out_acc_mps2` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.set_engage_speed` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.limit_speed` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.max_lateral_accel_mps2` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.limit_lateral_acceleration` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.min_limited_speed_mps` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.smooth_velocities` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.jerk_weight` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.over_v_weight` | `3000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.over_a_weight` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.over_j_weight` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.velocity_tracking_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_velocity_optimizer.continuous_jerk_smoother.accel_tracking_weight` | `300.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_velocity_optimizer.param.yaml` |
| `trajectory_extender.nearest_dist_threshold_m` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_extender.param.yaml` |
| `trajectory_extender.nearest_yaw_threshold_deg` | `60.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_extender.param.yaml` |
| `trajectory_extender.backward_trajectory_extension_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_extender.param.yaml` |
| `trajectory_point_fixer.remove_close_points` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_point_fixer.param.yaml` |
| `trajectory_point_fixer.resample_close_points` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_point_fixer.param.yaml` |
| `trajectory_point_fixer.min_dist_to_remove_m` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_point_fixer.param.yaml` |
| `trajectory_point_fixer.min_dist_to_resample_m` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_point_fixer.param.yaml` |
| `trajectory_point_fixer.stop_detection_velocity_threshold_mps` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_point_fixer.param.yaml` |
| `trajectory_mpt_optimizer.corridor_width_m` | `3.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.enable_adaptive_width` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.curvature_width_factor` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.velocity_width_factor` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.min_clearance_m` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.reset_previous_data_each_iteration` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.enable_debug_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.output_delta_arc_length_m` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.output_backward_traj_length_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.ego_nearest_dist_threshold_m` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg` | `45.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_mpt_optimizer.acceleration_moving_average_window` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.option.steer_limit_constraint` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.option.visualize_sampling_num` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.option.enable_manual_warm_start` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.option.enable_warm_start` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.option.enable_optimization_validation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.common.num_points` | `100` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.common.delta_arc_length` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.clearance.hard_clearance_from_road` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.clearance.soft_clearance_from_road` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.soft_collision_free_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.lat_error_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.yaw_error_weight` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.yaw_error_rate_weight` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.steer_input_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.steer_rate_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.terminal_lat_error_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.terminal_yaw_error_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.goal_lat_error_weight` | `1000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.weight.goal_yaw_error_weight` | `1000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.max_bound_fixing_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.max_longitudinal_margin_for_bound_violation` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.max_avoidance_cost` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.avoidance_cost_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.avoidance_cost_band_length` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.avoidance_cost_decrease_rate` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.min_drivable_width` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.weight.lat_error_weight` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.weight.yaw_error_weight` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.avoidance.weight.steer_input_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.option.l_inf_norm` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.option.soft_constraint` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.option.hard_constraint` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.method` | `"fitting_uniform_circle"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_calculation` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.front_radius_ratio` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.bicycle_model.rear_radius_ratio` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.uniform_circle.num` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.validation.max_lat_error` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `mpt.validation.max_yaw_error` | `1.046` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_mpt_optimizer.param.yaml` |
| `trajectory_qp_smoother.weight_smoothness` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.weight_fidelity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.time_step_s` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.osqp_eps_abs` | `1.0e-4` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.osqp_eps_rel` | `1.0e-4` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.osqp_max_iter` | `100` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.osqp_verbose` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.preserve_input_trajectory_orientation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.max_distance_for_orientation_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.use_velocity_based_fidelity` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.velocity_threshold_mps` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.sigmoid_sharpness` | `50.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.min_fidelity_weight` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.max_fidelity_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.num_constrained_points_start` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_qp_smoother.num_constrained_points_end` | `0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_qp_smoother.param.yaml` |
| `trajectory_spline_smoother.interpolation_resolution_m` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_spline_smoother.param.yaml` |
| `trajectory_spline_smoother.max_distance_discrepancy_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_spline_smoother.param.yaml` |
| `trajectory_spline_smoother.preserve_input_trajectory_orientation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_spline_smoother.param.yaml` |
| `trajectory_kinematic_feasibility.max_yaw_rate_rad_s` | `0.7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_kinematic_feasibility_enforcer.param.yaml` |
| `trajectory_kinematic_feasibility.time_step_s` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/plugins/trajectory_kinematic_feasibility_enforcer.param.yaml` |
| `common.output_delta_arc_length` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `common.output_backward_traj_length` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.option.enable_warm_start` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.option.enable_optimization_validation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.common.num_points` | `100` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.common.delta_arc_length` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.num_joint_points` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.clearance_for_fix` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.clearance_for_joint` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.clearance.clearance_for_smooth` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.weight.smooth_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.weight.lat_error_weight` | `0.001` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.qp.max_iteration` | `10000` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.qp.eps_abs` | `1.0e-7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.qp.eps_rel` | `1.0e-7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `elastic_band.validation.max_error` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `ego_nearest_dist_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `ego_nearest_yaw_threshold` | `1.046` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.max_path_shape_around_ego_lat_dist` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.max_path_shape_forward_lon_dist` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.max_path_shape_forward_lat_dist` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.max_ego_moving_dist` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.max_goal_moving_dist` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |
| `replan.max_delta_time_sec` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/config/trajectory_smoothing/elastic_band_smoother.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_core_planning`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_osqp_interface`, `autoware_path_optimizer`, `autoware_path_smoother`, `autoware_planning_msgs`, `autoware_qp_interface`, `autoware_trajectory`, `autoware_utils`, `autoware_utils_math`, `autoware_vehicle_info_utils`, `geometry_msgs`, `nav_msgs`, `pluginlib`, `rclcpp`, `rclcpp_components`, `tf2`, `tf2_geometry_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 103 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 34 |
| `pluginlib` | 24 |
| `parameters` | 80 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/README.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/docs/continuous_jerk_filter.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/docs/kinematic_feasibility_enforcer.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/docs/mpt_optimizer.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/docs/qp_smoother.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_optimizer/docs/trajectory_optimizer_specification.md`
