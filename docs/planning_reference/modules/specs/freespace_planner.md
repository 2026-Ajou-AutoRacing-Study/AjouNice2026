> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_freespace_planner

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_freespace_planner`
- Version: `0.51.0`
- Package description: The autoware_freespace_planner package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::freespace_planner::FreespacePlannerNode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |

## Purpose And Role

`freespace_planner_node` is a global path planner node that plans trajectory in the space having static/dynamic obstacles. This node is currently based on Hybrid A\* search algorithm in `freespace_planning_algorithms` package. Other algorithms such as rrt\* will be also added and selectable in the future. **Note** Due to the constraint of trajectory following, the output trajectory will be split to include only the single direction path. In other words, the output trajectory doesn't include both forward and backward trajectories at once.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/occupancy_grid` | `OccupancyGrid` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/include/autoware/freespace_planner/freespace_planner_node.hpp` |
| `~/input/scenario` | `Scenario` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/include/autoware/freespace_planner/freespace_planner_node.hpp` |
| `~/input/odometry` | `Odometry, autoware_utils::polling_policy::All` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/include/autoware/freespace_planner/freespace_planner_node.hpp` |
| `~/input/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `~/debug/pose_array` | `PoseArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `~/debug/partial_pose_array` | `PoseArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `is_completed` | `std_msgs::msg::Bool` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `planning_algorithm` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `waypoints_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `update_rate` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `th_arrived_distance_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `th_stopped_time_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `th_stopped_velocity_mps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `th_course_out_distance_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `th_obstacle_time_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `vehicle_shape_margin_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `replan_when_obstacle_found` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `replan_when_course_out` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `time_limit` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `theta_size` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `angle_goal_range` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `curve_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `reverse_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `direction_change_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `lateral_goal_range` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `longitudinal_goal_range` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `max_turning_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `turning_steps` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |
| `obstacle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `planning_algorithm` | `"astar"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `waypoints_velocity` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `update_rate` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `th_arrived_distance_m` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `th_stopped_time_sec` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `th_stopped_velocity_mps` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `th_course_out_distance_m` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `th_obstacle_time_sec` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `vehicle_shape_margin_m` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `replan_when_obstacle_found` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `replan_when_course_out` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `time_limit` | `30000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `max_turning_ratio` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `turning_steps` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `theta_size` | `120` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `angle_goal_range` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `lateral_goal_range` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `longitudinal_goal_range` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `curve_weight` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `reverse_weight` | `0.7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `direction_change_weight` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `obstacle_threshold` | `100` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.search_method` | `"forward"` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.only_behind_solutions` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.use_back` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.adapt_expansion_distance` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.expansion_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.near_goal_distance` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.distance_heuristic_weight` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.smoothness_weight` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.obstacle_distance_weight` | `1.75` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `astar.goal_lat_distance_weight` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `rrtstar.enable_update` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `rrtstar.use_informed_sampling` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `rrtstar.max_planning_time` | `150.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `rrtstar.neighbor_radius` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |
| `rrtstar.margin` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/config/freespace_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_freespace_planning_algorithms`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_route_handler`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_components`, `tf2`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 45 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 12 |
| `pluginlib` | 0 |
| `parameters` | 22 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_freespace_planner/README.md`
