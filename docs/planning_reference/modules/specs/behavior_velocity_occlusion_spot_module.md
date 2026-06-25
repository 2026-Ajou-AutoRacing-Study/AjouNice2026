> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_occlusion_spot_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_occlusion_spot_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::OcclusionSpotModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::OcclusionSpotModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |

## Purpose And Role

This module plans safe velocity to slow down before reaching collision point that hidden object is darting out from `occlusion spot` where driver can't see clearly because of obstacles. This module is activated if `launch_occlusion_spot` becomes true. To make pedestrian first zone map tag is one of the TODOs. This module is prototype implementation to care occlusion spot. To solve the excessive deceleration due to false positive of the perception, the logic of detection method can be selectable. This point has not been discussed in detail and needs to be improved.

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
| `occlusion_spot.detection_method` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.pass_judge` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.use_object_info` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.use_moving_object_ray_cast` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.use_partition_lanelet` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.pedestrian_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.pedestrian_radius` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.debug.is_show_occlusion` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.debug.is_show_cv_window` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.debug.is_show_processing_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.threshold.detection_area_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.threshold.detection_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.threshold.stuck_vehicle_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.threshold.lateral_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.threshold.search_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.threshold.search_angle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.safety_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.safe_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.max_slow_down_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.max_slow_down_accel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.non_effective_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.non_effective_acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.motion.min_allowed_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.detection_area.min_occlusion_spot_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.detection_area.min_longitudinal_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.detection_area.max_lateral_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.detection_area.slice_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.grid.free_space_max` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.grid.occupied_min` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/manager.cpp` |
| `occlusion_spot.detection_method` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.pass_judge` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.use_object_info` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.use_moving_object_ray_cast` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.use_partition_lanelet` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.pedestrian_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.pedestrian_radius` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.debug.is_show_occlusion` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.debug.is_show_cv_window` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.debug.is_show_processing_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.threshold.detection_area_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.threshold.detection_area_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.threshold.stuck_vehicle_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.threshold.lateral_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.threshold.search_dist` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.threshold.search_angle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.safety_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.safe_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.max_slow_down_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.max_slow_down_accel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.non_effective_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.non_effective_acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.motion.min_allowed_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.detection_area.min_occlusion_spot_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.detection_area.min_longitudinal_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.detection_area.max_lateral_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.detection_area.slice_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.grid.free_space_max` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |
| `occlusion_spot.grid.occupied_min` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `occlusion_spot.detection_method` | `"occupancy_grid"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.pass_judge` | `"smooth_velocity"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.use_object_info` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.use_moving_object_ray_cast` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.use_partition_lanelet` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.pedestrian_vel` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.pedestrian_radius` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.debug.is_show_occlusion` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.debug.is_show_cv_window` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.debug.is_show_processing_time` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.threshold.detection_area_offset` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.threshold.detection_area_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.threshold.stuck_vehicle_vel` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.threshold.lateral_distance` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.threshold.search_dist` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.threshold.search_angle` | `0.63` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.safety_ratio` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.max_slow_down_jerk` | `-0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.max_slow_down_accel` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.non_effective_jerk` | `-0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.non_effective_acceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.min_allowed_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.motion.safe_margin` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.detection_area.min_occlusion_spot_size` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.detection_area.slice_length` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.detection_area.min_longitudinal_offset` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.detection_area.max_lateral_distance` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.grid.free_space_max` | `43` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |
| `occlusion_spot.grid.occupied_min` | `57` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/config/occlusion_spot.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner_common`, `autoware_cmake`, `autoware_grid_map_utils`, `autoware_interpolation`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `grid_map_ros`, `libboost-dev`, `libopencv-dev`, `nav_msgs`, `pluginlib`, `rclcpp`, `tf2`, `tf2_geometry_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 31 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 10 |
| `pluginlib` | 5 |
| `parameters` | 59 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/README.md`
