> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_planner

## Identity

- Source set: `release_workspace`
- Repository: `core/autoware_core`
- Package path: `core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner`
- Version: `1.8.0`
- Package description: The autoware_behavior_velocity_planner package
- Primary README: `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| rclcpp_component | `autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |

## Purpose And Role

`behavior_velocity_planner` is a planner that adjust velocity based on the traffic rules. It loads modules as plugins. Please refer to the links listed below for detail on each module. When each module plans velocity, it considers based on `base_link`(center of rear-wheel axis) pose. So for example, in order to stop at a stop line with the vehicles' front on the stop line, it calculates `base_link` position from the distance between `base_link` to front and modifies path velocity from the `base_link` position.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/path_with_lane_id` | `autoware_internal_planning_msgs::msg::PathWithLaneId` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `~/input/dynamic_objects` | `autoware_perception_msgs::msg::PredictedObjects` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/no_ground_pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/vehicle_odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/vector_map` | `LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/external_velocity_limit_mps` | `VelocityLimit` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/node.hpp` |
| `~/input/dynamic_objects` | `autoware_perception_msgs::msg::PredictedObjects` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/no_ground_pointcloud` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/vehicle_odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/vector_map` | `LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/external_velocity_limit_mps` | `VelocityLimit` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/include/autoware/behavior_velocity_planner/experimental/node.hpp` |
| `~/input/path_with_lane_id` | `autoware_internal_planning_msgs::msg::PathWithLaneId` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/path` | `autoware_planning_msgs::msg::Path` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `~/debug/path` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `~/output/path` | `autoware_planning_msgs::msg::Path` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `~/debug/path` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `~/service/load_plugin` | `LoadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `~/service/unload_plugin` | `UnloadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `~/service/load_plugin` | `LoadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `~/service/unload_plugin` | `UnloadPlugin` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `forward_path_length` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `backward_path_length` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `behavior_output_path_interval` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `is_simulation` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp` |
| `forward_path_length` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `backward_path_length` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `behavior_output_path_interval` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |
| `is_simulation` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/experimental/node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `forward_path_length` | `1000.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/config/behavior_velocity_planner.param.yaml` |
| `backward_path_length` | `5.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/config/behavior_velocity_planner.param.yaml` |
| `behavior_output_path_interval` | `1.0` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/config/behavior_velocity_planner.param.yaml` |
| `planning_factor_console_output.enable` | `false` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/config/behavior_velocity_planner.param.yaml` |
| `planning_factor_console_output.duration` | `1000` | `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/config/behavior_velocity_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner_common`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_trajectory`, `autoware_utils_debug`, `autoware_utils_logging`, `autoware_utils_pcl`, `autoware_utils_rclcpp`, `autoware_velocity_smoother`, `diagnostic_msgs`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `libboost-dev`, `pcl_conversions`, `pluginlib`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 57 |
| `lifecycle` | 0 |
| `component` | 5 |
| `tf2` | 17 |
| `pluginlib` | 5 |
| `parameters` | 14 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/README.md`
