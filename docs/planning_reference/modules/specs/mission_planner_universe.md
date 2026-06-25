> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_mission_planner_universe

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_mission_planner_universe`
- Version: `0.51.0`
- Package description: The autoware_mission_planner_universe package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::mission_planner_universe::RouteSelector` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| rclcpp_component | `autoware::mission_planner_universe::MissionPlanner` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| rclcpp_component | `autoware::mission_planner_universe::GoalPoseVisualizer` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/goal_pose_visualizer/goal_pose_visualizer.cpp` |
| pluginlib_plugin | `autoware::mission_planner_universe::lanelet2::DefaultPlanner -> autoware::mission_planner_universe::PlannerPlugin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |

## Purpose And Role

`Mission Planner` calculates a route that navigates from the current ego pose to the goal pose following the given check points. The route is made of a sequence of lanes on a static map. Dynamic objects (e.g. pedestrians and other vehicles) and dynamic map information (e.g. road construction which blocks some lanes) are not considered during route planning. Therefore, the output topic is only published when the goal pose or check points are given and will be latched until the new goal pose or check points are given.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/planner/state` | `RouteState` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/planner/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/input/operation_mode_state` | `OperationModeState` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/input/vector_map` | `LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/input/modified_goal` | `PoseWithUuidStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/input/reroute_availability` | `RerouteAvailability` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.hpp` |
| `input/route` | `autoware_planning_msgs::msg::LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/goal_pose_visualizer/goal_pose_visualizer.cpp` |
| `~/input/vector_map` | `LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/main/state` | `RouteState` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/main/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/mrm/state` | `RouteState` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/mrm/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/debug/route_marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/state` | `RouteState` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `output/goal_pose` | `geometry_msgs::msg::PoseStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/goal_pose_visualizer/goal_pose_visualizer.cpp` |
| `~/debug/goal_footprint` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `~/main/clear_route` | `ClearRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/main/set_waypoint_route` | `SetWaypointRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/main/set_lanelet_route` | `SetLaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/mrm/clear_route` | `ClearRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/mrm/set_waypoint_route` | `SetWaypointRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/mrm/set_lanelet_route` | `SetLaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/planner/clear_route` | `ClearRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/planner/set_lanelet_route` | `SetLaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/planner/set_waypoint_route` | `SetWaypointRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/route_selector.cpp` |
| `~/clear_route` | `ClearRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/set_lanelet_route` | `SetLaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/set_preferred_primitive` | `autoware_planning_msgs::srv::SetPreferredPrimitive` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `~/set_waypoint_route` | `SetWaypointRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `map_frame` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `reroute_time_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `minimum_reroute_length` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `allow_reroute_in_autonomous_mode` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `goal_lanelet_transparency` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp` |
| `arrival_check_angle_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/arrival_checker.cpp` |
| `arrival_check_lateral_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/arrival_checker.cpp` |
| `arrival_check_longitudinal_undershoot_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/arrival_checker.cpp` |
| `arrival_check_longitudinal_overshoot_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/arrival_checker.cpp` |
| `arrival_check_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/arrival_checker.cpp` |
| `goal_angle_threshold_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |
| `enable_correct_goal_pose` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |
| `consider_no_drivable_lanes` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |
| `check_footprint_inside_lanes` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `map_frame` | `map` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `arrival_check_angle_deg` | `45.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `arrival_check_lateral_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `arrival_check_longitudinal_undershoot_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `arrival_check_longitudinal_overshoot_distance` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `arrival_check_duration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `goal_angle_threshold_deg` | `45.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `enable_correct_goal_pose` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `reroute_time_threshold` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `minimum_reroute_length` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `consider_no_drivable_lanes` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `check_footprint_inside_lanes` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `allow_reroute_in_autonomous_mode` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |
| `goal_lanelet_transparency` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/config/mission_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_adapi_v1_msgs`, `autoware_cmake`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_planning_msgs`, `autoware_qos_utils`, `autoware_route_handler`, `autoware_test_utils`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `pluginlib`, `rclcpp`, `rclcpp_components`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 85 |
| `lifecycle` | 0 |
| `component` | 9 |
| `tf2` | 16 |
| `pluginlib` | 5 |
| `parameters` | 14 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_mission_planner_universe/README.md`
