> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_boundary_departure_prevention_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module`
- Version: `0.51.0`
- Package description: boundary departure prevention feature in motion_velocity_planner
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::experimental::BoundaryDeparturePreventionModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/boundary_departure_prevention_module.cpp` |

## Purpose And Role

The Boundary Departure Prevention Module is experimental. It subscribes to the control module’s predicted path and steering report, creating a circular dependency. This violates Autoware’s design principle of forward-only data flow, where control depends on planning, not the other way round. As a result, this module is not officially supported and will remain unofficial for the foreseeable future. This module inserts slow down points and publishes error diagnostics when the ego vehicle is near or about to cross an uncrossable boundary, such as a road border.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/boundary_departure_prevention_module.cpp` |
| `~/` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/boundary_departure_prevention_module.cpp` |
| `~/debug/` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/boundary_departure_prevention_module.cpp` |
| `~/debug/processing_time_detail_ms/` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/boundary_departure_prevention_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `<module_name>boundary_types_to_detect` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_point_merge_distance_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_pt_shift.dist_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_pt_shift.angle_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_pt_shift.goal_dist_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_cutoff_time_s.predicted_path` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_cutoff_time_s.near_boundary` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_cutoff_time_s.departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>on_time_buffer_s.critical_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>on_time_buffer_s.near_boundary` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>off_time_buffer_s.critical_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>off_time_buffer_s.near_boundary` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<module_name>th_max_lateral_query_num` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_normal_abnormality>enable` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_localization_abnormality>enable` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_longitudinal_abnormality>enable` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<footprint_envelop_ns>lat_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<footprint_envelop_ns>lon_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns>enable` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns>steering_rate_velocities_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns>steering_rate_limits_rps` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns>delay_s` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns>offset_rps` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns>factor` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<localization_footprint_envelop_ns>lat_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<localization_footprint_envelop_ns>lon_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<lon_tracking_ns>scale` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<lon_tracking_ns>extra_margin_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_diag>near_boundary` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_diag>approaching_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_diag>critical_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dpt_type>slow_down_near_boundary` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dpt_type>stop_before_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dpt_type>slow_down_before_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_vel>min` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_vel>max` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_acc>min` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_acc>max` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_jerk>min` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_jerk>max` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_th_trigger>brake_delay_s` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_th_trigger>dist_error_m` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dist_to_bound_left>min` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dist_to_bound_left>max` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dist_to_bound_right>min` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |
| `<ns_dist_to_bound_right>max` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/parameters.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `boundary_departure_prevention.boundary_types_to_detect` | `[road_border]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_max_lateral_query_num` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_point_merge_distance_m` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_pt_shift.dist_m` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_pt_shift.angle_deg` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_pt_shift.goal_dist_m` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_cutoff_time_s.predicted_path` | `3.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_cutoff_time_s.near_boundary` | `3.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.th_cutoff_time_s.departure` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.on_time_buffer_s.near_boundary` | `0.15` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.on_time_buffer_s.critical_departure` | `0.15` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.off_time_buffer_s.near_boundary` | `0.15` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.off_time_buffer_s.critical_departure` | `0.15` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.normal.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.normal.footprint_envelop.lon_m` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.normal.footprint_envelop.lat_m` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.localization.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.localization.footprint_envelop.lon_m` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.localization.footprint_envelop.lat_m` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.longitudinal.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.longitudinal.lon_tracking.scale` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.longitudinal.lon_tracking.extra_margin_m` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_accelerated.enable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_accelerated.steering_rate_velocities_mps` | `[0.0, 3.5, 9.1, 14.7, 20.3, 25.9, 31.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_accelerated.steering_rate_limits_rps` | `[3.14, 0.777, 0.115, 0.044, 0.023, 0.014, 0.010]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_accelerated.delay_s` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_accelerated.offset_rps` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_accelerated.factor` | `1.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_stuck.enable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_stuck.steering_rate_velocities_mps` | `[0.0, 3.5, 9.1, 14.7, 20.3, 25.9, 31.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_stuck.steering_rate_limits_rps` | `[3.14, 0.777, 0.115, 0.044, 0.023, 0.014, 0.010]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_stuck.delay_s` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_stuck.offset_rps` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_stuck.factor` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_left.enable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_left.steering_rate_velocities_mps` | `[0.0, 3.5, 9.1, 14.7, 20.3, 25.9, 31.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_left.steering_rate_limits_rps` | `[3.14, 0.777, 0.115, 0.044, 0.023, 0.014, 0.010]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_left.delay_s` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_left.offset_rps` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_left.factor` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_right.enable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_right.steering_rate_velocities_mps` | `[0.0, 3.5, 9.1, 14.7, 20.3, 25.9, 31.5]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_right.steering_rate_limits_rps` | `[3.14, 0.777, 0.115, 0.044, 0.023, 0.014, 0.010]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_right.delay_s` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_right.offset_rps` | `-0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.abnormality.steering_sudden_right.factor` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.diagnostic.near_boundary` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.diagnostic.approaching_departure` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.diagnostic.critical_departure` | `2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.enable.slow_down_near_boundary` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.enable.slow_down_before_departure` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.enable.stop_before_departure` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_vel_kmph.min` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_vel_kmph.max` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_acc_mps2.min` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_acc_mps2.max` | `-2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_jerk_mps3.min` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_jerk_mps3.max` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.brake_delay_s` | `1.3` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.dist_error_m` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_dist_to_boundary_m.left.min` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_dist_to_boundary_m.left.max` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_dist_to_boundary_m.right.min` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |
| `boundary_departure_prevention.slow_down_behavior.th_trigger.th_dist_to_boundary_m.right.max` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/config/boundary_departure_prevention.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_deprecated_boundary_departure_checker`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_trajectory`, `autoware_utils`, `autoware_utils_rclcpp`, `autoware_utils_system`, `autoware_utils_uuid`, `autoware_vehicle_info_utils`, `fmt`, `geometry_msgs`, `pluginlib`, `range-v3`, `rclcpp`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 26 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 1 |
| `pluginlib` | 3 |
| `parameters` | 47 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/README.md`
