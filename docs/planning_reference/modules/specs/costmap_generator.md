> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_costmap_generator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_costmap_generator`
- Version: `0.51.0`
- Package description: The autoware_costmap_generator package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::costmap_generator::CostmapGenerator` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/src/costmap_generator.cpp` |

## Purpose And Role

This node reads `PointCloud` and/or `DynamicObjectArray` and creates an `OccupancyGrid` and `GridMap`. `VectorMap(Lanelet2)` is optional. None 1. Execute the command `source install/setup.bash` to setup the environment 2. Run `ros2 launch costmap_generator costmap_generator.launch.xml` to launch the node

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/src/costmap_generator.cpp` |
| `~/input/points_no_ground` | `sensor_msgs::msg::PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/include/autoware/costmap_generator/costmap_generator.hpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/include/autoware/costmap_generator/costmap_generator.hpp` |
| `~/input/scenario` | `autoware_internal_planning_msgs::msg::Scenario` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/include/autoware/costmap_generator/costmap_generator.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/grid_map` | `grid_map_msgs::msg::GridMap` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/src/costmap_generator.cpp` |
| `~/output/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/src/costmap_generator.cpp` |
| `processing_time` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/src/costmap_generator.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/src/costmap_generator.cpp` |

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
| `costmap_frame` | `"map"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `vehicle_frame` | `"base_link"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `map_frame` | `"map"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `update_rate` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `activate_by_scenario` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_min_value` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_max_value` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_resolution` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_length_x` | `70.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_length_y` | `70.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_position_x` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `grid_position_y` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `maximum_lidar_height_thres` | `2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `minimum_lidar_height_thres` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `use_wayarea` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `use_parkinglot` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `use_objects` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `use_points` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `expand_polygon_size` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |
| `size_of_expansion_kernel` | `9` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/config/costmap_generator.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_grid_map_utils`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_perception_msgs`, `autoware_test_utils`, `autoware_utils`, `generate_parameter_library`, `grid_map_ros`, `libpcl-all-dev`, `pcl_conversions`, `pcl_ros`, `rclcpp`, `rclcpp_components`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 35 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 29 |
| `pluginlib` | 0 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_costmap_generator/README.md`
