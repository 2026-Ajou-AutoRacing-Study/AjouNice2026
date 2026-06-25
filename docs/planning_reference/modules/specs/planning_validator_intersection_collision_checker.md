> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_validator_intersection_collision_checker

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker`
- Version: `0.51.0`
- Package description: The autoware_planning_validator_intersection_collision_checker package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::planning_validator::IntersectionCollisionChecker -> autoware::planning_validator::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/src/intersection_collision_checker.cpp` |

## Purpose And Role

The `intersection_collision_checker` is a plugin module of `autoware_planning_validator` node. It is responsible for validating the planning trajectory at intersections by verifying that it does NOT lead to a collision with other road vehicles. The check is executed only when: - Ego is approaching a `turn_direction` lane - Ego trajectory intersects with lanes other than `route_lanelets` The intersection_collision_checker checks for collisions using pointcloud data and route information. It identifies target lanes at intersections and extracts pcd objects withing target lanes, and performs simplistic tracking and velocity estimation of pcd objects for each target lane. Times to arrival are co

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/intersection_collision_checker/debug/cluster_points` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/src/intersection_collision_checker.cpp` |
| `~/intersection_collision_checker/debug/voxel_points` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/src/intersection_collision_checker.cpp` |
| `~/intersection_collision_checker/debug/state` | `StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/src/intersection_collision_checker.cpp` |

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
| `icc_parameters.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.detection_range` | `50.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.ttc_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.ego_deceleration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.min_time_horizon` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.on_time_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.off_time_buffer` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.close_distance_th` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.close_time_th` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.right_turn.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.right_turn.check_traffic_signal` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.right_turn.check_crossing_lanes` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.right_turn.check_turning_lanes` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.right_turn.crossing_lane_angle_th` | `0.785398` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.left_turn.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.left_turn.check_traffic_signal` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.left_turn.check_turning_lanes` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.height_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.min_height` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.voxel_grid_filter.x` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.voxel_grid_filter.y` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.voxel_grid_filter.z` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.voxel_grid_filter.min_size` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.clustering.tolerance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.clustering.min_height` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.clustering.min_size` | `10` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.clustering.max_size` | `10000` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.velocity_estimation.max_acceleration` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.velocity_estimation.max_velocity` | `25.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.velocity_estimation.observation_time` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.velocity_estimation.max_history_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.velocity_estimation.buffer_size` | `10` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.pointcloud.latency` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.filter.min_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |
| `icc_parameters.filter.moving_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/config/intersection_collision_checker.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_validator`, `autoware_route_handler`, `autoware_signal_processing`, `autoware_traffic_light_utils`, `autoware_trajectory`, `autoware_utils`, `generate_parameter_library`, `libboost-dev`, `pluginlib`, `rclcpp`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 31 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 2 |
| `pluginlib` | 3 |
| `parameters` | 1 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/README.md`
