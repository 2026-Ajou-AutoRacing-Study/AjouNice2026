> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_road_user_stop_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module`
- Version: `0.51.0`
- Package description: road_user_stop module to stop ego vehicle when road users (pedestrians, cyclists) are on the road
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::RoadUserStopModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/road_user_stop_module.cpp` |

## Purpose And Role

The `road_user_stop` module stops the ego vehicle when pedestrians, cyclists, or other road users are detected on or near the road within the ego's planned trajectory. This module ensures safe interaction with vulnerable road users by maintaining appropriate stopping distances and making decisions about when to stop versus when to continue driving. This module is activated if the launch parameter `launch_road_user_stop_module` is set to true. - If object tracking is lost for an extended period or object classification is misidentified, the module may not function properly and may fail to stop as expected

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/road_user_stop/virtual_walls` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/road_user_stop_module.cpp` |
| `~/road_user_stop/debug_markers` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/road_user_stop_module.cpp` |
| `~/debug/road_user_stop/trajectory` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/road_user_stop_module.cpp` |
| `~/debug/processing_time_detail_ms/road_user_stop` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/road_user_stop_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `normal.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `normal.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `normal.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `normal.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `limit.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `limit.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `limit.max_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |
| `limit.min_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/src/types.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `road_user_stop.option.suppress_sudden_stop` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.longitudinal_margin.default_margin` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.longitudinal_margin.terminal_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.longitudinal_margin.minimum_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.opposing_traffic.stop_margin` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.opposing_traffic.max_negative_velocity` | `-0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.opposing_traffic.min_velocity_for_stop_planning` | `2.77` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.opposing_traffic.effective_deceleration` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.hold_stop_velocity_threshold` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.hold_stop_distance_threshold` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.stop_on_curve.enable_approaching` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.stop_on_curve.additional_stop_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.stop_on_curve.min_stop_margin` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.limit_min_acc` | `-2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.sudden_object_acc_threshold` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.sudden_object_dist_threshold` | `1000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stop_planning.abandon_to_stop` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.object_type.pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.object_type.bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.object_type.motorcycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.object_type.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.trajectory_lateral_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.opposing_traffic_detection.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.opposing_traffic_detection.angle_threshold` | `150.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.opposing_traffic_detection.min_speed_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.intersection.exclude` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.crosswalk.exclude` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.crosswalk.margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.min_detection_duration` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.lost_object_retention_duration` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.obstacle_filtering.polygon_expansion_length` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stopped_object_tracking.stopped_velocity_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stopped_object_tracking.stopped_position_tolerance` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stopped_object_tracking.ego_reached_wall_distance_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |
| `road_user_stop.stopped_object_tracking.stopped_duration_after_ego_arrival` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/config/road_user_stop.param.yaml` |

## Dependencies

`ament_cmake`, `ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils_geometry`, `autoware_utils_math`, `autoware_utils_system`, `autoware_utils_uuid`, `autoware_utils_visualization`, `autoware_vehicle_info_utils`, `generate_parameter_library`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `tf2_geometry_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 38 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 4 |
| `pluginlib` | 3 |
| `parameters` | 9 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/README.md`
