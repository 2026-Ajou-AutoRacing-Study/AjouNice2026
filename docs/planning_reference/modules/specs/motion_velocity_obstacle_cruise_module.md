> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_obstacle_cruise_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module`
- Version: `0.51.0`
- Package description: obstacle cruise feature in motion_velocity_planner
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::ObstacleCruiseModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |

## Purpose And Role

The `obstacle_cruise` module does the cruise planning against a dynamic obstacle in front of the ego. This module is activated if the launch parameter `launch_obstacle_cruise_module` is set to true. The obstacles meeting the following condition are determined as obstacles for cruising. - The lateral distance from the object to the ego's trajectory is smaller than `obstacle_filtering.max_lat_margin`. - The object type is for cruising according to `obstacle_filtering.object_type.*`.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `/planning/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/obstacle_cruise/virtual_walls` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |
| `~/obstacle_cruise/debug_markers` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |
| `~/debug/obstacle_cruise/planning_info` | `Float32MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |
| `~/debug/processing_time_detail_ms/obstacle_cruise` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |
| `~/debug/obstacle_cruise/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |
| `~/optimized_sv_trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `~/optimized_st_graph` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `~/boundary` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `~/debug/wall_marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `obstacle_cruise.option.planning_algorithm` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp` |
| `normal.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `normal.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `normal.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `normal.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `limit.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `limit.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `limit.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `limit.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.object_type.pointcloud` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.max_lat_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.crossing_obstacle.obstacle_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.crossing_obstacle.obstacle_traj_angle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.obstacle_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.ego_obstacle_overlap_time_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.max_prediction_time_for_collision_check` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.max_lateral_time_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.num_of_predicted_paths` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.yield.enable_yield` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.yield.lat_distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.yield.max_lat_dist_between_obstacles` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.yield.max_obstacles_collision_time` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.yield.stopped_obstacle_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.obstacle_velocity_threshold_from_cruise` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.obstacle_filtering.obstacle_velocity_threshold_to_cruise` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.cruise_planning.idling_time` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.cruise_planning.min_ego_accel_for_rss` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.cruise_planning.min_object_accel_for_rss` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.cruise_planning.safe_distance_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/parameters.hpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.min_accel_during_cruise` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.use_velocity_limit_based_planner` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kp` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.ki` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kd` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.output_ratio_` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.vel_to_acc_` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.enable_` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.disable_` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.enable_` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.min_cruise_target_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.time_to_evaluate_rss` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.error_function_type` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.pid_based_planner.lpf_normalized_error_cruise_dist_gain` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/pid_based_planner/pid_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.dense_resampling_time_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.sparse_resampling_time_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.dense_time_horizon` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.max_time_horizon` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.t_dangerous` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.velocity_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.replan_vel_deviation` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.engage_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.engage_acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.engage_exit_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.stop_dist_to_prohibit_engage` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.max_s_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.max_v_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_s_safety_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_s_ideal_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_v_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_a_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_j_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/optimization_based_planner/optimization_based_planner.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `obstacle_cruise.option.planning_algorithm` | `"pid_base"` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.idling_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.min_ego_accel_for_rss` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.min_object_accel_for_rss` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.safe_distance_margin` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.use_velocity_limit_based_planner` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.error_function_type` | `quadratic` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kp` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.ki` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kd` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.output_ratio_during_accel` | `0.6` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.vel_to_acc_weight` | `12.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.enable_jerk_limit_to_output_acc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.disable_target_acceleration` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_acc` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_acc` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_acc` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_jerk` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_jerk` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_jerk` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_acc_ratio_during_accel` | `0.6` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_jerk_ratio_during_accel` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.enable_jerk_limit_to_output_acc` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.min_accel_during_cruise` | `-2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.min_cruise_target_vel` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.time_to_evaluate_rss` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.pid_based_planner.lpf_normalized_error_cruise_dist_gain` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.dense_resampling_time_interval` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.sparse_resampling_time_interval` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.dense_time_horizon` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.max_time_horizon` | `25.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.velocity_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.t_dangerous` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.replan_vel_deviation` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.engage_velocity` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.engage_acceleration` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.engage_exit_ratio` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.stop_dist_to_prohibit_engage` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.max_s_weight` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.max_v_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_s_safety_weight` | `1000000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_s_ideal_weight` | `50.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_v_weight` | `500000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_a_weight` | `5000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.cruise_planning.optimization_based_planner.over_j_weight` | `10000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.unknown` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.inside.pedestrian` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.bicycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.outside.pedestrian` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.bicycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.object_type.side_stopped.pedestrian` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.max_lat_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.crossing_obstacle.obstacle_velocity_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.crossing_obstacle.obstacle_traj_angle_threshold` | `0.523599` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.obstacle_velocity_threshold` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.ego_obstacle_overlap_time_threshold` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.max_prediction_time_for_collision_check` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.max_lateral_time_margin` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.outside_obstacle.num_of_predicted_paths` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.yield.enable_yield` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.yield.lat_distance_threshold` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.yield.max_lat_dist_between_obstacles` | `2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.yield.max_obstacles_collision_time` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.yield.stopped_obstacle_velocity_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.obstacle_velocity_threshold_from_cruise` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |
| `obstacle_cruise.obstacle_filtering.obstacle_velocity_threshold_to_cruise` | `3.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/config/obstacle_cruise.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_osqp_interface`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_signal_processing`, `autoware_utils`, `autoware_utils_geometry`, `autoware_vehicle_info_utils`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 79 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 2 |
| `pluginlib` | 3 |
| `parameters` | 70 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/README.md`
