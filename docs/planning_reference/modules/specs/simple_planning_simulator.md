> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_simple_planning_simulator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/simulator/autoware_simple_planning_simulator`
- Version: `0.51.0`
- Package description: simple_planning_simulator as a ROS 2 node
- Primary README: `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::simulator::simple_planning_simulator::SimplePlanningSimulator` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |

## Purpose And Role

This node simulates the vehicle motion for a vehicle command in 2D using a simple vehicle model. The purpose of this simulator is for the integration test of planning and control modules. This does not simulate sensing or perception, but is implemented in pure c++ only and works without GPU. - It simulates only in 2D motion. - It does not perform physical operations such as collision and sensing, but only calculates the integral results of vehicle dynamics.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `input/vector_map` | `LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/initialpose` | `PoseWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/initialtwist` | `TwistStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/manual_ackermann_control_command` | `Control` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/gear_command` | `GearCommand` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/manual_gear_command` | `GearCommand` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/turn_indicators_command` | `TurnIndicatorsCommand` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/hazard_lights_command` | `HazardLightsCommand` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/engage` | `Engage` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/actuation_command` | `ActuationCommandStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `input/ackermann_control_command` | `Control` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `output/control_mode_report` | `ControlModeReport` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/gear_report` | `GearReport` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/turn_indicators_report` | `TurnIndicatorsReport` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/hazard_lights_report` | `HazardLightsReport` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/pose` | `PoseWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/twist` | `VelocityReport` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/imu` | `Imu` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `/tf` | `tf2_msgs::msg::TFMessage` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/actuation_status` | `ActuationStatusStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `output/steering` | `SteeringReport` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `input/control_mode_request` | `ControlModeCommand` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `simulated_frame_id` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `origin_frame_id` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `add_measurement_noise` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `initial_engage_state` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `enable_road_slope_simulation` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `enable_pub_steer` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vehicle_model_type` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `timer_sampling_time_ms` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `initialize_source` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `pos_noise_stddev` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vel_noise_stddev` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `rpy_noise_stddev` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_noise_stddev` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `x_stddev` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `y_stddev` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vel_lim` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vel_rate_lim` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_lim` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_rate_lim` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `acc_time_delay` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `acc_time_constant` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vel_time_delay` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vel_time_constant` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_time_delay` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_time_constant` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_dead_band` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_bias` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `debug_acc_scaling_factor` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `debug_steer_scaling_factor` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `acceleration_map_path` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `accel_time_delay` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `accel_time_constant` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `brake_time_delay` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `brake_time_constant` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `convert_accel_cmd` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `convert_brake_cmd` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `accel_map_path` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `brake_map_path` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vgr_coef_a` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vgr_coef_b` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `vgr_coef_c` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |
| `steer_map_path` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | publisher | `input/ackermann_control_command` | `Control` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp` |
| test | publisher | `input/actuation_command` | `ActuationCommandStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp` |
| test | publisher | `input/initialpose` | `PoseWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp` |
| test | publisher | `input/gear_command` | `GearCommand` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp` |
| test | subscription | `output/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `cg_to_front_m` | `1.228` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `cg_to_rear_m` | `1.5618` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `front_corner_stiffness` | `17000.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `rear_corner_stiffness` | `20000.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `mass_kg` | `1460.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `yaw_inertia_kgm2` | `2170.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `width_m` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `front_overhang_m` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `rear_overhang_m` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `max_steer_angle` | `0.70` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml` |
| `simulated_frame_id` | `"base_link"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `origin_frame_id` | `"map"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `vehicle_model_type` | `"DELAY_STEER_ACC_GEARED"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `initialize_source` | `"INITIAL_POSE_TOPIC"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `timer_sampling_time_ms` | `25` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `add_measurement_noise` | `False` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `vel_lim` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `vel_rate_lim` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `steer_lim` | `0.6` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `steer_rate_lim` | `6.28` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `acc_time_delay` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `acc_time_constant` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `steer_time_delay` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `steer_time_constant` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `steer_dead_band` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `x_stddev` | `0.0001` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `y_stddev` | `0.0001` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `enable_road_slope_simulation` | `true` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml` |
| `simulated_frame_id` | `"base_link"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `origin_frame_id` | `"map"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `vehicle_model_type` | `"ACTUATION_CMD_MECHANICAL"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `initialize_source` | `"INITIAL_POSE_TOPIC"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `timer_sampling_time_ms` | `25` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `add_measurement_noise` | `False` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `enable_road_slope_simulation` | `True` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `vel_lim` | `50.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `vel_rate_lim` | `7.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `steer_lim` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `steer_rate_lim` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `acc_time_delay` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `acc_time_constant` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `steer_time_delay` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `steer_time_constant` | `0.001` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `steer_dead_band` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `x_stddev` | `0.0001` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `y_stddev` | `0.0001` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `accel_time_delay` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `accel_time_constant` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `brake_time_delay` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `brake_time_constant` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `accel_map_path` | `$(find-pkg-share simple_planning_simulator)/data/actuation_cmd_map/accel_map.csv` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `brake_map_path` | `$(find-pkg-share simple_planning_simulator)/data/actuation_cmd_map/brake_map.csv` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `convert_accel_cmd` | `true` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `convert_brake_cmd` | `true` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `convert_steer_cmd_method` | `"vgr"` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `vgr_coef_a` | `15.713` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `vgr_coef_b` | `0.053` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `vgr_coef_c` | `0.042` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `enable_pub_steer` | `true` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.kp` | `386.9151820510161` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.ki` | `5.460970982628869` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.kd` | `0.03550834077602694` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.ff_gain` | `0.03051963576179274` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.angle_limit` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.rate_limit` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.dead_zone_threshold` | `0.00708241866710033` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_a` | `0.15251276182076065` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_b` | `-0.17301900674117585` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_c` | `1.5896528355739639` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_d` | `0.002300899817071436` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_e` | `-0.0418928856764797` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_f` | `0.18449047960081838` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_g` | `-0.06320887302605509` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.poly_h` | `0.18696796150634806` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.inertia` | `25.17844747941984` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.damping` | `117.00653795106054` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.stiffness` | `0.17526182368541224` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.friction` | `0.6596571248682918` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.vgr_coef_a` | `2.4181735349544224` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.vgr_coef_b` | `-0.013434076966833082` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.vgr_coef_c` | `-0.033963661615283594` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.steering_torque_limit` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |
| `mechanical_params.torque_delay_time` | `0.0007641271506616108` | `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_index_cpp`, `ament_lint_auto`, `autoware_cmake`, `autoware_control_msgs`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_learning_based_vehicle_model`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_utils_geometry`, `autoware_utils_rclcpp`, `autoware_vehicle_info_utils`, `autoware_vehicle_msgs`, `geometry_msgs`, `lanelet2_core`, `launch_ros`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `tf2`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_vehicle_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 72 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 15 |
| `pluginlib` | 0 |
| `parameters` | 69 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md`
- `sources/autoware_workspace/universe/autoware_universe/simulator/autoware_simple_planning_simulator/docs/actuation_cmd_sim.md`
