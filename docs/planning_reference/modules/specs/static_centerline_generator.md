> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_static_centerline_generator

## Identity

- Source set: `release_workspace`
- Repository: `tools`
- Package path: `tools/planning/autoware_static_centerline_generator`
- Version: `0.7.0`
- Package description: The autoware_static_centerline_generator package
- Primary README: `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| executable_main | `main()` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/main.cpp` |

## Purpose And Role

This package statically calculates the centerline satisfying path footprints inside the drivable area. On narrow-road driving, the default centerline, which is the middle line between lanelets' right and left boundaries, often causes path footprints outside the drivable area. To make path footprints inside the drivable area, we use online path shape optimization by [the autoware_path_optimizer package](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/autoware_path_optimizer/).

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `/static_centerline_generator/road_boundary_lateral_margin` | `std_msgs::msg::Float32` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `/static_centerline_generator/traj_start_index` | `std_msgs::msg::Int32` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `/static_centerline_generator/traj_end_index` | `std_msgs::msg::Int32` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `/static_centerline_generator/save_map` | `std_msgs::msg::Empty` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `/static_centerline_generator/validate` | `std_msgs::msg::Empty` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `lanelet2_map_topic` | `LaneletMapBin` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/output/whole_centerline` | `Trajectory` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/output/centerline` | `Trajectory` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/output/map_saved` | `std_msgs::msg::Empty` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/validation_results` | `MarkerArray` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/debug/markers` | `MarkerArray` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/debug/ego_footprint_bounds` | `MarkerArray` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `~/input_centerline` | `PathWithLaneId` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/centerline_source/optimization_trajectory_based_centerline.cpp` |
| `~/debug/raw_centerline` | `Path` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/centerline_source/optimization_trajectory_based_centerline.cpp` |
| `~/debug/iterative_path` | `Path` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/centerline_source/optimization_trajectory_based_centerline.cpp` |
| `~/debug/iterative_smoothed_trajectory` | `Trajectory` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/centerline_source/optimization_trajectory_based_centerline.cpp` |
| `~/debug/iterative_optimized_trajectory` | `Trajectory` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/centerline_source/optimization_trajectory_based_centerline.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `/planning/static_centerline_generator/load_map` | `LoadMap` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `/planning/static_centerline_generator/plan_route` | `PlanRoute` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `/planning/static_centerline_generator/plan_path` | `PlanPath` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `centerline_source` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `lanelet2_input_file_path` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `start_lanelet_id` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `end_lanelet_id` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `output_trajectory_interval` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/static_centerline_generator_node.cpp` |
| `mode` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/main.cpp` |
| `bag_filename` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/src/centerline_source/bag_ego_trajectory_based_centerline.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `wheel_radius` | `0.39` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `wheel_width` | `0.42` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `wheel_base` | `2.74` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `wheel_tread` | `1.63` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `front_overhang` | `1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `rear_overhang` | `1.03` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `left_overhang` | `0.1` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `right_overhang` | `0.1` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `vehicle_height` | `2.5` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `max_steer_angle` | `0.70` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/vehicle_info.param.yaml` |
| `max_vel` | `11.1` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `normal.min_acc` | `-1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `normal.max_acc` | `1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `normal.min_jerk` | `-1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `normal.max_jerk` | `1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `limit.min_acc` | `-2.5` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `limit.max_acc` | `1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `limit.min_jerk` | `-1.5` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `limit.max_jerk` | `1.5` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/common.param.yaml` |
| `ego_nearest_dist_threshold` | `3.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/nearest_search.param.yaml` |
| `ego_nearest_yaw_threshold` | `1.046` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/nearest_search.param.yaml` |
| `marker_color` | `["FF0000", "FF5A00", "FFFF00"]` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/static_centerline_generator.param.yaml` |
| `marker_color_dist_thresh` | `[0.1, 0.2, 0.3]` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/static_centerline_generator.param.yaml` |
| `output_trajectory_interval` | `1.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/static_centerline_generator.param.yaml` |
| `validation.dist_threshold_to_road_border` | `0.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/static_centerline_generator.param.yaml` |
| `validation.max_steer_angle_margin` | `0.0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/static_centerline_generator.param.yaml` |
| `debug.wait_time_during_planning_iteration` | `0` | `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/config/static_centerline_generator.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_gmock`, `ament_lint_auto`, `autoware_behavior_path_goal_planner_module`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_behavior_velocity_planner`, `autoware_cmake`, `autoware_geography_utils`, `autoware_global_parameter_loader`, `autoware_interpolation`, `autoware_lanelet2_extension`, `autoware_lanelet2_map_visualizer`, `autoware_launch`, `autoware_lint_common`, `autoware_map_loader`, `autoware_map_msgs`, `autoware_map_projection_loader`, `autoware_map_tf_generator`, `autoware_mission_planner_universe`, `autoware_motion_utils`, `autoware_osqp_interface`, `autoware_path_generator`, `autoware_path_optimizer`, `autoware_path_smoother`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_qos_utils`, `autoware_route_handler`, `autoware_sample_vehicle_description`, `autoware_universe_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `glog`, `libqt5-core`, `libqt5-widgets`, `python3-flask-cors`, `rclcpp`, `rclcpp_components`, `ros_testing`, `rosidl_default_generators`, `rosidl_default_runtime`, `tier4_map_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 80 |
| `lifecycle` | 0 |
| `component` | 1 |
| `tf2` | 0 |
| `pluginlib` | 2 |
| `parameters` | 10 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/tools/planning/autoware_static_centerline_generator/README.md`
