> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_surround_obstacle_checker

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_surround_obstacle_checker`
- Version: `0.51.0`
- Package description: The autoware_surround_obstacle_checker package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::surround_obstacle_checker::SurroundObstacleCheckerNode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.cpp` |

## Purpose And Role

This module subscribes required data (ego-pose, obstacles, etc), and publishes zero velocity limit to keep stopping if any of stop conditions are satisfied. Check that `surround_obstacle_checker` receives no ground pointcloud, dynamic objects and current velocity data. Calculate distance between ego vehicle and the nearest object. In this function, it calculates the minimum distance between the polygon of ego vehicle and all points in pointclouds and the polygons of dynamic objects.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.hpp` |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.hpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/marker` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/debug_marker.cpp` |
| `~/debug/footprint` | `PolygonStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/debug_marker.cpp` |
| `~/debug/footprint_offset` | `PolygonStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/debug_marker.cpp` |
| `~/debug/footprint_recover_offset` | `PolygonStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/debug_marker.cpp` |
| `~/output/velocity_limit_clear_command` | `VelocityLimitClearCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.cpp` |
| `~/output/max_velocity` | `VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/src/node.cpp` |

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
| test | publisher | `/surround_obstacle_checker_node/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/test/test_planning_factor.cpp` |
| test | publisher | `/localization/kinematic_state` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/test/test_planning_factor.cpp` |
| test | publisher | `/surround_obstacle_checker_node/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/test/test_planning_factor.cpp` |
| test | publisher | `/surround_obstacle_checker_node/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/test/test_planning_factor.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `pointcloud.enable_check` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pointcloud.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pointcloud.surround_check_side_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pointcloud.surround_check_back_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `unknown.enable_check` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `unknown.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `unknown.surround_check_side_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `unknown.surround_check_back_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `car.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `car.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `car.surround_check_side_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `car.surround_check_back_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `truck.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `truck.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `truck.surround_check_side_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `truck.surround_check_back_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bus.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bus.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bus.surround_check_side_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bus.surround_check_back_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `trailer.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `trailer.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `trailer.surround_check_side_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `trailer.surround_check_back_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `motorcycle.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `motorcycle.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `motorcycle.surround_check_side_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `motorcycle.surround_check_back_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bicycle.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bicycle.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bicycle.surround_check_side_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `bicycle.surround_check_back_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pedestrian.enable_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pedestrian.surround_check_front_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pedestrian.surround_check_side_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `pedestrian.surround_check_back_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `surround_check_hysteresis_distance` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `state_clear_time` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `stop_state_ego_speed` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `publish_debug_footprints` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |
| `debug_footprint_label` | `"car"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml` |

## Dependencies

`ament_cmake`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_test_utils`, `autoware_utils`, `autoware_vehicle_info_utils`, `diagnostic_msgs`, `eigen`, `eigen3_cmake_module`, `generate_parameter_library`, `libpcl-all-dev`, `pcl_conversions`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 60 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 15 |
| `pluginlib` | 0 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/README.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_surround_obstacle_checker/surround_obstacle_checker-design.ja.md`
