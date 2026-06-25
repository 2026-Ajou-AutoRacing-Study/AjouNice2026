> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_obstacle_slow_down_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module`
- Version: `0.51.0`
- Package description: obstacle slow down feature in motion_velocity_planner
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::ObstacleSlowDownModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/obstacle_slow_down_module.cpp` |

## Purpose And Role

The `obstacle_slow_down` module does the slow down planning when there is a static/dynamic obstacle near the trajectory. This module is activated if the launch parameter `launch_obstacle_slow_down_module` is set to true. Obstacles meeting the following condition are determined as obstacles for slowing down. - The object type is for slowing down according to `obstacle_filtering.object_type.*`. - The lateral distance from the object to the ego's trajectory is smaller than `obstacle_filtering.max_lat_margin` and larger than `obstacle_filtering.min_lat_margin`.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/obstacle_slow_down/virtual_walls` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/obstacle_slow_down_module.cpp` |
| `~/obstacle_slow_down/debug_markers` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/obstacle_slow_down_module.cpp` |
| `~/debug/obstacle_slow_down/planning_info` | `Float32MultiArrayStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/obstacle_slow_down_module.cpp` |
| `~/debug/processing_time_detail_ms/obstacle_slow_down` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/obstacle_slow_down_module.cpp` |
| `~/debug/obstacle_slow_down/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/obstacle_slow_down_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `normal.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `normal.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `normal.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `normal.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `limit.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `limit.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `limit.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `limit.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.object_type.pointcloud` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.min_lat_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.max_lat_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.lat_hysteresis_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.max_lat_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.successive_num_to_entry_slow_down_condition` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.obstacle_filtering.successive_num_to_exit_slow_down_condition` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `<const std::string & ns>.{<object_label>|default}.<const std::string & param>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `<ns><object_label>.<param>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `<ns>default.<param>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.slow_down_min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.slow_down_min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.lpf_gain_slow_down_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.lpf_gain_lateral_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.lpf_gain_dist_to_slow_down` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.time_margin_on_target_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.moving_object_speed_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.moving_object_hysteresis_range` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params..{<object_label>|default}.<to_param_name(side, motion, "min_lat_margin")>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params..{<object_label>|default}.<to_param_name(side, motion, "max_lat_margin")>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params..{<object_label>|default}.<to_param_name(side, motion, "min_ego_velocity")>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params..{<object_label>|default}.<to_param_name(side, motion, "max_ego_velocity")>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params..{<object_label>|default}.wheel_off_track_scale` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/src/parameters.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `obstacle_slow_down.slow_down_planning.slow_down_min_acc` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.slow_down_min_jerk` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.lpf_gain_slow_down_vel` | `0.9` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.lpf_gain_lateral_distance` | `0.7` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.lpf_gain_dist_to_slow_down` | `0.7` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.time_margin_on_target_velocity` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.moving.min_lat_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.moving.max_lat_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.moving.min_ego_velocity` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.moving.max_ego_velocity` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.static.min_lat_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.static.max_lat_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.static.min_ego_velocity` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.left.static.max_ego_velocity` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.moving.min_lat_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.moving.max_lat_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.moving.min_ego_velocity` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.moving.max_ego_velocity` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.static.min_lat_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.static.max_lat_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.static.min_ego_velocity` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.right.static.max_ego_velocity` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.object_type_specified_params.default.wheel_off_track_scale` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.moving_object_speed_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.slow_down_planning.moving_object_hysteresis_range` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.object_type.pointcloud` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.min_lat_margin` | `-10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.max_lat_margin` | `1.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.lat_hysteresis_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.max_lat_velocity` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.successive_num_to_entry_slow_down_condition` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |
| `obstacle_slow_down.obstacle_filtering.successive_num_to_exit_slow_down_condition` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/config/obstacle_slow_down.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_signal_processing`, `autoware_utils`, `autoware_utils_geometry`, `autoware_vehicle_info_utils`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 33 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 2 |
| `pluginlib` | 3 |
| `parameters` | 29 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module/README.md`
