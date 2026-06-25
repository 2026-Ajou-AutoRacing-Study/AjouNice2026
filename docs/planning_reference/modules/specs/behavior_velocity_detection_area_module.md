> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_velocity_detection_area_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_velocity_detection_area_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_velocity_planner::DetectionAreaModulePlugin -> autoware::behavior_velocity_planner::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| pluginlib_plugin | `autoware::behavior_velocity_planner::experimental::DetectionAreaModulePlugin -> autoware::behavior_velocity_planner::experimental::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |

## Purpose And Role

If pointcloud or predicted objects are detected in a detection area defined on a map, the stop planning will be executed at the predetermined point. This module is activated when there is a detection area on the target lane. 1. Gets a detection area and stop line from map information and confirms if there are obstacles in the detection area 2. Inserts stop point l[m] in front of the stop line 3. Calculates required braking distance based on current velocity, `max_deceleration`, and `delay_response_time`

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
| `detection_area.stop_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.use_dead_line` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.dead_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.state_clear_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.hold_stop_margin_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.distance_to_judge_over_stop_line` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.suppress_pass_judge_when_stopping` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.enable_detected_obstacle_logging` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.unstoppable_policy` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.max_deceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.delay_response_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.pointcloud` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.animal` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.hazard` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.over_drivable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.target_filtering.under_drivable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/manager.cpp` |
| `detection_area.stop_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.use_dead_line` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.dead_line_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.state_clear_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.hold_stop_margin_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.distance_to_judge_over_stop_line` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.suppress_pass_judge_when_stopping` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.enable_detected_obstacle_logging` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.pointcloud` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.animal` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.hazard` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.over_drivable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.target_filtering.under_drivable` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.unstoppable_policy` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.max_deceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |
| `detection_area.delay_response_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/src/experimental/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `detection_area.stop_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.pointcloud` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.car` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.truck` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.bus` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.trailer` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.motorcycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.bicycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.pedestrian` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.animal` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.hazard` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.over_drivable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.target_filtering.under_drivable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.use_dead_line` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.dead_line_margin` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.unstoppable_policy` | `"stop_after_stopline"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.max_deceleration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.delay_response_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.state_clear_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.hold_stop_margin_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.distance_to_judge_over_stop_line` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.suppress_pass_judge_when_stopping` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |
| `detection_area.enable_detected_obstacle_logging` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/config/detection_area.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_velocity_planner`, `autoware_behavior_velocity_planner_common`, `autoware_behavior_velocity_rtc_interface`, `autoware_cmake`, `autoware_lanelet2_extension`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 55 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 3 |
| `pluginlib` | 5 |
| `parameters` | 49 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/README.md`
