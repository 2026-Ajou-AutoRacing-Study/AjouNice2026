> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_planner

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner`
- Version: `0.51.0`
- Package description: The behavior_path_planner package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::behavior_path_planner::BehaviorPathPlannerNode` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |

## Purpose And Role

The Behavior Path Planner's main objective is to significantly enhance the safety of autonomous vehicles by minimizing the risk of accidents. It improves driving efficiency through time conservation and underpins reliability with its rule-based approach. Additionally, it allows users to integrate their own custom behavior modules or use it with different types of vehicles, such as cars, buses, and delivery robots, as well as in various environments, from busy urban streets to open highways.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/route` | `LaneletRoute, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/vector_map` | `LaneletMapBin, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/accel` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/scenario` | `Scenario` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/perception` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/occupancy_grid_map` | `OccupancyGrid` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/costmap` | `OccupancyGrid` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/traffic_signals` | `TrafficLightGroupArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `~/input/lateral_offset` | `LateralOffset` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `/system/operation_mode/state` | `OperationModeState` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |
| `/planning/scenario_planning/max_velocity` | `autoware_internal_planning_msgs::msg::VelocityLimit` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/behavior_path_planner_node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/path` | `PathWithLaneId` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/output/turn_indicators_cmd` | `TurnIndicatorsCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/output/hazard_lights_cmd` | `HazardLightsCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/output/modified_goal` | `PoseWithUuidStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/output/is_reroute_available` | `RerouteAvailability` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/debug/avoidance_debug_message_array` | `AvoidanceDebugMsgArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/debug/turn_signal_info` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `~/debug/bound` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `planning_hz` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `cyclic_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |
| `enable_traffic_signal_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `external_request_lane_change_left.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `external_request_lane_change_left.enable_simultaneous_execution_as_approved_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `external_request_lane_change_left.enable_simultaneous_execution_as_candidate_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `external_request_lane_change_right.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `external_request_lane_change_right.enable_simultaneous_execution_as_approved_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `external_request_lane_change_right.enable_simultaneous_execution_as_candidate_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `lane_change_left.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `lane_change_left.enable_simultaneous_execution_as_approved_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `lane_change_left.enable_simultaneous_execution_as_candidate_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `lane_change_right.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `lane_change_right.enable_simultaneous_execution_as_approved_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `lane_change_right.enable_simultaneous_execution_as_candidate_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `start_planner.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `start_planner.enable_simultaneous_execution_as_approved_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `start_planner.enable_simultaneous_execution_as_candidate_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `side_shift.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `side_shift.enable_simultaneous_execution_as_approved_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `side_shift.enable_simultaneous_execution_as_candidate_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `goal_planner.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `goal_planner.enable_simultaneous_execution_as_approved_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `goal_planner.enable_simultaneous_execution_as_candidate_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `static_obstacle_avoidance.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `static_obstacle_avoidance.enable_simultaneous_execution_as_approved_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `static_obstacle_avoidance.enable_simultaneous_execution_as_candidate_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `avoidance_by_lane_change.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `avoidance_by_lane_change.enable_simultaneous_execution_as_approved_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `avoidance_by_lane_change.enable_simultaneous_execution_as_candidate_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `dynamic_obstacle_avoidance.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `dynamic_obstacle_avoidance.enable_simultaneous_execution_as_approved_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `dynamic_obstacle_avoidance.enable_simultaneous_execution_as_candidate_module` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `sampling_planner.enable_rtc` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `sampling_planner.enable_simultaneous_execution_as_approved_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `sampling_planner.enable_simultaneous_execution_as_candidate_module` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/scene_module_manager.param.yaml` |
| `drivable_area_right_bound_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `drivable_area_left_bound_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `drivable_area_types_to_skip` | `[road_border]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.enabled` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.print_runtime` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.max_expansion_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.min_bound_interval` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.smoothing.curvature_average_window` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.smoothing.max_bound_rate` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.smoothing.arc_length_range` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.ego.extra_wheelbase` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.ego.extra_front_overhang` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.ego.extra_width` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.exclude_static` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.exclude_dynamic` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.th_stopped_object_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.safety_margin.front` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.safety_margin.rear` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.safety_margin.left` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.object_exclusion.safety_margin.right` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.path_preprocessing.max_arc_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.path_preprocessing.resample_interval` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.path_preprocessing.reuse_max_deviation` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `dynamic_expansion.avoid_linestring.distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/drivable_area_expansion.param.yaml` |
| `traffic_light_signal_timeout` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `cyclic_timeout` | `0.90` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `enable_traffic_signal_timeout` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `planning_hz` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `backward_path_length` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `forward_path_length` | `300.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `backward_length_buffer_for_end_of_pull_over` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `backward_length_buffer_for_end_of_pull_out` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `minimum_pull_over_length` | `16.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `refine_goal_search_radius_range` | `7.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_intersection_search_distance` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_intersection_angle_threshold_deg` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_roundabout_on_entry` | `"None"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_roundabout_entry_indicator_persistence` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_roundabout_on_exit` | `"Left"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_roundabout_search_distance` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_roundabout_angle_threshold_deg` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_roundabout_backward_depth` | `50` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_minimum_search_distance` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_path_backward_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_search_time` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_shift_length_threshold` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_remaining_shift_length_threshold` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_remaining_distance_to_bound_threshold` | `1.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `turn_signal_on_swerving` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `enable_akima_spline_first` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `enable_cog_on_centerline` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `input_path_interval` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |
| `output_path_interval` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_adapi_v1_msgs`, `autoware_behavior_path_planner_common`, `autoware_cmake`, `autoware_freespace_planning_algorithms`, `autoware_frenet_planner`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_path_sampler`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_signal_processing`, `autoware_utils`, `autoware_utils_diagnostics`, `autoware_vehicle_info_utils`, `autoware_vehicle_msgs`, `behaviortree_cpp_v3`, `diagnostic_msgs`, `geometry_msgs`, `libboost-dev`, `libopencv-dev`, `magic_enum`, `pluginlib`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 44 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 2 |
| `pluginlib` | 3 |
| `parameters` | 6 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/README.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/docs/behavior_path_planner_interface_design.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/docs/behavior_path_planner_limitations.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/docs/behavior_path_planner_manager_design.md`
