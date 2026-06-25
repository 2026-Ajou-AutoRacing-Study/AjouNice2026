> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_dynamic_obstacle_stop_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module`
- Version: `0.51.0`
- Package description: dynamic_obstacle_stop module to stop ego when in the immediate path of a dynamic object
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::DynamicObstacleStopModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |

## Purpose And Role

`dynamic_obstacle_stop` is a module that stops the ego vehicle from entering the _immediate_ path of a dynamic object. The _immediate_ path of an object is the area that the object would traverse during a given time horizon, assuming constant velocity and heading. This module is activated if the launch parameter `launch_dynamic_obstacle_stop_module` is set to true in the motion planning launch file. The module insert a stop point where the ego trajectory collides with the immediate path of an object.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `~/debug/` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `<ns_>.extra_object_width` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.minimum_object_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.stop_distance_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.time_horizon` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.hysteresis` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.add_stop_duration_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.remove_stop_duration_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.minimum_object_distance_from_ego_trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |
| `<ns_>.ignore_unavoidable_collisions` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/src/dynamic_obstacle_stop_module.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `dynamic_obstacle_stop.extra_object_width` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.minimum_object_velocity` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.stop_distance_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.time_horizon` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.hysteresis` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.add_stop_duration_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.remove_stop_duration_buffer` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.minimum_object_distance_from_ego_trajectory` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |
| `dynamic_obstacle_stop.ignore_unavoidable_collisions` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/config/dynamic_obstacle_stop.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 20 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 7 |
| `pluginlib` | 3 |
| `parameters` | 10 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module/README.md`
