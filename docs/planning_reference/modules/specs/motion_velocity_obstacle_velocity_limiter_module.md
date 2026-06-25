> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_obstacle_velocity_limiter_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module`
- Version: `0.51.0`
- Package description: Package to adjust velocities of a trajectory in order for the ride to feel safe
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::ObstacleVelocityLimiterModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/src/obstacle_velocity_limiter_module.cpp` |
| executable_main | `main()` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/benchmarks/collision_checker_benchmark.cpp` |

## Purpose And Role

This node limits the velocity when driving in the direction of an obstacle. For example, it allows to reduce the velocity when driving close to a guard rail in a curve. Using a parameter `min_ttc` (minimum time to collision), the node set velocity limits such that no collision with an obstacle would occur, even without new control inputs for a duration of `min_ttc`. To achieve this, the motion of the ego vehicle is simulated forward in time at each point of the trajectory to create a corresponding footprint.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/src/obstacle_velocity_limiter_module.cpp` |
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/src/obstacle_velocity_limiter_module.cpp` |
| `~/debug/` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/src/obstacle_velocity_limiter_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `distance_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/src/obstacle_velocity_limiter_module.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `min_ttc` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `distance_buffer` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `min_adjusted_velocity` | `2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `max_deceleration` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `trajectory_preprocessing.start_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `trajectory_preprocessing.max_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `trajectory_preprocessing.max_duration` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `trajectory_preprocessing.downsample_factor` | `10` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `trajectory_preprocessing.calculate_steering_angles` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `simulation.model` | `particle` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `simulation.distance_method` | `exact` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `simulation.steering_offset` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `simulation.nb_points` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.dynamic_source` | `static_only` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.ignore_obstacles_on_path` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.ignore_extra_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.occupancy_grid_threshold` | `60` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.dynamic_obstacles_buffer` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.dynamic_obstacles_min_vel` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.filter_envelope` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.rtree_min_points` | `500` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |
| `obstacles.rtree_min_segments` | `1600` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/config/obstacle_velocity_limiter.param.yaml` |

## Dependencies

`ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_grid_map_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_utils`, `autoware_vehicle_info_utils`, `eigen`, `eigen3_cmake_module`, `grid_map_msgs`, `grid_map_ros`, `lanelet2_core`, `libboost-dev`, `nav_msgs`, `pcl_ros`, `pluginlib`, `rclcpp`, `sensor_msgs`, `tf2_eigen`, `tf2_geometry_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 23 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 7 |
| `pluginlib` | 3 |
| `parameters` | 23 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/README.md`
