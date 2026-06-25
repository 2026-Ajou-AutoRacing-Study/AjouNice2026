> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_validator_trajectory_checker

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker`
- Version: `0.51.0`
- Package description: The autoware_planning_validator_trajectory_checker package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::planning_validator::TrajectoryChecker -> autoware::planning_validator::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |

## Purpose And Role

The `intersection_collision_checker` is a plugin module of `autoware_planning_validator` node. It is responsible for validating the planning trajectory before it is published. The following features are supported for trajectory validation and can have thresholds set by parameters: - **Invalid field** : e.g. Inf, Nan - **Trajectory points interval** : invalid if any of the distance of trajectory points is too large - **Curvature** : invalid if the trajectory has too sharp turns that is not feasible for the given vehicle kinematics

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
| `default_handling_type` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |
| `trajectory_checker.yaw_deviation.th_trajectory_yaw_shift` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |
| `trajectory_checker.trajectory_shift.lat_shift_th` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |
| `trajectory_checker.trajectory_shift.forward_shift_th` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |
| `trajectory_checker.trajectory_shift.backward_shift_th` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |
| `trajectory_checker.forward_trajectory_length.acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |
| `trajectory_checker.forward_trajectory_length.margin` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | publisher | `/planning_validator_node/input/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | publisher | `/planning_validator_node/input/kinematics` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | publisher | `/planning_validator_node/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | publisher | `/planning_validator_node/input/pointcloud` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | publisher | `/planning_validator_node/input/operational_mode_state` | `OperationModeState` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | publisher | `/planning_validator_node/input/lanelet_map_bin` | `LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | publisher | `/planning_validator_node/input/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |
| test | subscription | `/diagnostics` | `DiagnosticArray` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/test/test_trajectory_checker_diag.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `trajectory_checker.interval.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.interval.threshold` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.curvature.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.curvature.threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.relative_angle.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.relative_angle.threshold` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.lateral_accel.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.lateral_accel.threshold` | `9.8` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.min_lon_accel.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.min_lon_accel.threshold` | `-9.8` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.max_lon_accel.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.max_lon_accel.threshold` | `9.8` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.lateral_jerk.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.lateral_jerk.threshold` | `7.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.steering.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.steering.threshold` | `1.414` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.steering_rate.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.steering_rate.threshold` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.distance_deviation.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.distance_deviation.threshold` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.lon_distance_deviation.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.lon_distance_deviation.threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.velocity_deviation.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.velocity_deviation.threshold` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.yaw_deviation.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.yaw_deviation.threshold` | `1.5708` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.yaw_deviation.th_trajectory_yaw_shift` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.forward_trajectory_length.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.forward_trajectory_length.acceleration` | `-3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.forward_trajectory_length.margin` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.trajectory_shift.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.trajectory_shift.lat_shift_th` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.trajectory_shift.forward_shift_th` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.trajectory_shift.backward_shift_th` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.trajectory_shift.handling_type` | `2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |
| `trajectory_checker.trajectory_shift.override_error_diag` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `angles`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_test_manager`, `autoware_planning_validator`, `autoware_planning_validator_test_utils`, `autoware_trajectory`, `autoware_utils`, `autoware_vehicle_info_utils`, `pluginlib`, `rclcpp`, `rclcpp_components`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 39 |
| `lifecycle` | 0 |
| `component` | 1 |
| `tf2` | 6 |
| `pluginlib` | 3 |
| `parameters` | 12 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker/README.md`
