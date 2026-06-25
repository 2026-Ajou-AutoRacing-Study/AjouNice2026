> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_dynamic_obstacle_avoidance_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_dynamic_obstacle_avoidance_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::DynamicObstacleAvoidanceModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/src/manager.cpp` |

## Purpose And Role

This module is under development. This module provides avoidance functions for vehicles, pedestrians, and obstacles in the vicinity of the ego's path in combination with the [autoware_path_optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/). Each module performs the following roles. Dynamic Avoidance module cuts off the drivable area according to the position and velocity of the target to be avoided.

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
| `dynamic_avoidance.common.enable_debug_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.common.use_hatched_road_markings` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.max_obstacle_vel` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.min_obstacle_vel` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.successive_num_to_entry_dynamic_avoidance_condition` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.successive_num_to_exit_dynamic_avoidance_condition` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.min_obj_lat_offset_to_ego_path` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.max_obj_lat_offset_to_ego_path` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.cut_in_object.min_time_to_start_cut_in` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.cut_in_object.min_lon_offset_ego_to_object` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.cut_in_object.min_object_vel` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.cut_out_object.max_time_from_outside_ego_path` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.cut_out_object.min_object_lat_vel` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.cut_out_object.min_object_vel` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.crossing_object.min_overtaking_object_vel` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.crossing_object.max_overtaking_object_angle` | `1.05` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.crossing_object.min_oncoming_object_vel` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.crossing_object.max_oncoming_object_angle` | `0.523` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.crossing_object.max_pedestrian_crossing_vel` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.front_object.max_object_angle` | `0.785` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.front_object.min_object_vel` | `-0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.front_object.max_ego_path_lat_cover_ratio` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.target_object.stopped_object.max_object_vel` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.expand_drivable_area` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.polygon_generation_method` | `"ego_path_base"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.object_path_base.min_longitudinal_polygon_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.lat_offset_from_obstacle` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.margin_distance_around_pedestrian` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.predicted_path.end_time_to_consider` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.predicted_path.threshold_confidence` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.max_lat_offset_to_avoid` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.max_time_for_object_lat_shift` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.lpf_gain_for_lat_avoid_to_offset` | `0.9` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.max_ego_lat_acc` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.max_ego_lat_jerk` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.delay_time_ego_shift` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.overtaking_object.max_time_to_collision` | `40.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.overtaking_object.start_duration_to_avoid` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.overtaking_object.end_duration_to_avoid` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.overtaking_object.duration_to_hold_avoidance` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.oncoming_object.max_time_to_collision` | `40.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.oncoming_object.start_duration_to_avoid` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |
| `dynamic_avoidance.drivable_area_generation.oncoming_object.end_duration_to_avoid` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/config/dynamic_obstacle_avoidance.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_signal_processing`, `autoware_utils`, `autoware_vehicle_msgs`, `eigen3_cmake_module`, `geometry_msgs`, `lanelet2_core`, `pluginlib`, `rclcpp`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 25 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 8 |
| `pluginlib` | 3 |
| `parameters` | 51 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/README.md`
