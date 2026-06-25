> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_goal_planner_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_goal_planner_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::GoalPlannerModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/manager.cpp` |

## Purpose And Role

goal_planner generates a smooth path toward the goal and additionally searches for safe path and goal to execute dynamic pull_over on the road shoulders lanes following the traffic rules. If goal modification is not allowed, just park at the designated fixed goal using `fixed_goal_planner`. If allowed, `rough_goal_planner` works to park around the vacant spots in the shoulder lanes around the goal by executing pull_over toward left or right side of the road lanes.

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
| `goal_planner.th_arrived_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.th_stopped_velocity` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.th_stopped_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.center_line_path_interval` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.goal_priority` | `"minimum_weighted_distance"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.minimum_weighted_distance.lateral_weight` | `40.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.prioritize_goals_before_objects` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.parking_policy` | `"left_side"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.forward_goal_search_length` | `40.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.backward_goal_search_length` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.goal_search_interval` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.longitudinal_margin` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.max_lateral_offset` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.lateral_offset_interval` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.ignore_distance_from_lane_start` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.margin_from_boundary` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.high_curvature_threshold` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.bus_stop_area.use_bus_stop_area` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.bus_stop_area.goal_search_interval` | `0.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.goal_search.bus_stop_area.lateral_offset_interval` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.occupancy_grid.use_occupancy_grid_for_goal_search` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.occupancy_grid.use_occupancy_grid_for_goal_longitudinal_margin` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.occupancy_grid.use_occupancy_grid_for_path_collision_check` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.occupancy_grid.occupancy_grid_collision_check_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.occupancy_grid.theta_size` | `360` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.occupancy_grid.obstacle_threshold` | `60` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.collision_check_soft_margins` | `[5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.collision_check_hard_margins` | `[0.6]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.object_recognition_collision_check_max_extra_stopping_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.collision_check_outer_margin_factor` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.th_moving_object_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.detection_bound_offset` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.outer_road_detection_offset` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.object_recognition.inner_road_detection_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_decision_state_controller.check_collision_duration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.minimum_request_length` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.approximate_pull_over_distance` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.pull_over_prepare_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.pull_over_velocity` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.pull_over_minimum_velocity` | `1.38` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.decide_path_distance` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.maximum_deceleration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.maximum_jerk` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.low_velocity_threshold` | `1.38` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.stopping_distance_buffer` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.path_priority` | `"efficient_path"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.efficient_path_order` | `["SHIFT", "ARC_FORWARD", "ARC_BACKWARD"]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.lane_departure_check_expansion_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.shift_parking.enable_shift_parking` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.shift_parking.shift_sampling_num` | `4` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.shift_parking.maximum_lateral_jerk` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.shift_parking.minimum_lateral_jerk` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.shift_parking.deceleration_interval` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.shift_parking.after_shift_straight_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.max_steer_angle_margin_scale` | `0.72` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.forward.enable_arc_forward_parking` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.forward.after_forward_parking_straight_distance` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.forward.forward_parking_velocity` | `1.38` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.forward.forward_parking_lane_departure_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.forward.forward_parking_path_interval` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.backward.enable_arc_backward_parking` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.backward.after_backward_parking_straight_distance` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.backward.backward_parking_velocity` | `-1.38` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.backward.backward_parking_lane_departure_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.parallel_parking.backward.backward_parking_path_interval` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.enable_freespace_parking` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.freespace_parking_algorithm` | `"astar"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.vehicle_shape_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.time_limit` | `3000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.max_turning_ratio` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.turning_steps` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.theta_size` | `120` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.angle_goal_range` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.lateral_goal_range` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.longitudinal_goal_range` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.curve_weight` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.reverse_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.search_configs.direction_change_weight` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.costmap_configs.obstacle_threshold` | `30` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.search_method` | `"forward"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.only_behind_solutions` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.use_back` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.adapt_expansion_distance` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.expansion_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.near_goal_distance` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.distance_heuristic_weight` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.smoothness_weight` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.obstacle_distance_weight` | `1.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.astar.goal_lat_distance_weight` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.rrtstar.enable_update` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.rrtstar.use_informed_sampling` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.rrtstar.max_planning_time` | `150.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.rrtstar.neighbor_radius` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.freespace_parking.rrtstar.margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.bezier_parking.pull_over_angle_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.bezier_parking.after_shift_straight_distance` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.bezier_parking.lateral_acceleration_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.pull_over.bezier_parking.lateral_acceleration_filtering_duration` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.stop_condition.maximum_deceleration_for_stop` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.stop_condition.maximum_jerk_for_stop` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.min_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.min_acceleration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.max_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.time_horizon_for_front_object` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.time_horizon_for_rear_object` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.time_resolution` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.ego_predicted_path.delay_until_departure` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.safety_check_time_horizon` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.safety_check_time_resolution` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_check_forward_distance` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_check_backward_distance` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.ignore_object_velocity_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_types_to_check.check_unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_lane_configuration.check_current_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_lane_configuration.check_right_side_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_lane_configuration.check_left_side_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_lane_configuration.check_shoulder_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.object_lane_configuration.check_other_lane` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.include_opposite_lane` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.invert_opposite_lane` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.check_all_predicted_path` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.use_all_predicted_path` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.target_filtering.use_predicted_path_outside_lanelet` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.enable_safety_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.method` | `"integral_predicted_polygon"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.keep_unsafe_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.publish_debug_marker` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.rss_params.rear_vehicle_reaction_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.rss_params.rear_vehicle_safety_time_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.rss_params.lateral_distance_max_threshold` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.rss_params.longitudinal_distance_min_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.rss_params.longitudinal_velocity_delta_time` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.integral_predicted_polygon_params.forward_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.integral_predicted_polygon_params.backward_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.integral_predicted_polygon_params.lat_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.integral_predicted_polygon_params.time_horizon` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.hysteresis_factor_expand_rate` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.collision_check_yaw_diff_threshold` | `3.1416` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.backward_path_length` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.path_safety_check.safety_check_params.forward_path_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |
| `goal_planner.debug.print_debug_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/config/goal_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_index_cpp`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_bezier_sampler`, `autoware_cmake`, `autoware_freespace_planning_algorithms`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_rtc_interface`, `autoware_test_utils`, `autoware_trajectory`, `autoware_utils`, `eigen3_cmake_module`, `pluginlib`, `range-v3`, `rclcpp`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 124 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 8 |
| `pluginlib` | 3 |
| `parameters` | 141 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/README.md`
