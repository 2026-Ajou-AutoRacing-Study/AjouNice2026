> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_start_planner_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_start_planner_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::StartPlannerModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/manager.cpp` |

## Purpose And Role

This module generates and plans a path for safely merging from the shoulder lane or side of road lane into the center of the road lane. Specifically, it includes the following features: - Plan the path to automatically start from the shoulder lane or side of road lane to center of road lane. - When parked vehicles are present on the shoulder lane, the module generates a path that allows for starting with a gap of a specified margin. - If a collision with other traffic participants is detected while traveling on the generated path, it will stop as much as possible.

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
| `start_planner.th_arrived_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.th_stopped_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.th_stopped_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.prepare_time_before_start` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.th_distance_to_middle_of_the_road` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.skip_rear_vehicle_check` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.extra_width_margin_for_rear_obstacle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.collision_check_margins` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.collision_check_margin_from_front_object` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.th_moving_object_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.center_line_path_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.check_shift_path_lane_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.allow_check_shift_path_lane_departure_override` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.shift_collision_check_distance_from_end` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.minimum_shift_pull_out_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.minimum_shift_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.lateral_acceleration_sampling_num` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.lateral_jerk` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.maximum_lateral_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.minimum_lateral_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.maximum_curvature` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.end_pose_curvature_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.maximum_longitudinal_deviation` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.geometric_collision_check_distance_from_end` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.divide_pull_out_path` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.geometric_pull_out_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.arc_path_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.lane_departure_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.lane_departure_check_expansion_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.geometric_pull_out_max_steer_angle_margin_scale` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.turn_signal_on_centerline_start` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.clothoid_initial_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.clothoid_acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.clothoid_max_steer_angles_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.clothoid_max_steer_angle_rate_deg_per_sec` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.clothoid_collision_check_distance_from_end` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.check_clothoid_path_lane_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.search_priority` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.search_policy` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.enable_back` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.backward_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.max_back_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.backward_search_resolution` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.backward_path_update_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.ignore_distance_from_lane_end` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.stop_condition.maximum_deceleration_for_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.stop_condition.maximum_jerk_for_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.object_types_to_check_for_path_generation.check_pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.enable_freespace_planner` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.freespace_planner_algorithm` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.end_pose_search_start_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.end_pose_search_end_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.end_pose_search_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.vehicle_shape_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.time_limit` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.max_turning_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.turning_steps` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.search_configs.theta_size` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.search_configs.angle_goal_range` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.search_configs.curve_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.search_configs.reverse_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.search_configs.lateral_goal_range` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.search_configs.longitudinal_goal_range` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.costmap_configs.obstacle_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.astar.search_method` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.astar.only_behind_solutions` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.astar.use_back` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.astar.distance_heuristic_weight` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.rrtstar.enable_update` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.rrtstar.use_informed_sampling` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.rrtstar.max_planning_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.rrtstar.neighbor_radius` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.freespace_planner.rrtstar.margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<ego_path_ns>min_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<ego_path_ns>min_acceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<ego_path_ns>time_horizon_for_front_object` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<ego_path_ns>time_horizon_for_rear_object` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<ego_path_ns>time_resolution` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<ego_path_ns>delay_until_departure` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>safety_check_time_horizon` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>safety_check_time_resolution` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>object_check_forward_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>object_check_backward_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>ignore_object_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>include_opposite_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>invert_opposite_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>check_all_predicted_path` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>use_all_predicted_path` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_filter_ns>use_predicted_path_outside_lanelet` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_car` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_truck` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_bus` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_trailer` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_unknown` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_bicycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_motorcycle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_types_ns>check_pedestrian` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_lane_ns>check_current_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_lane_ns>check_right_side_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_lane_ns>check_left_side_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_lane_ns>check_shoulder_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<obj_lane_ns>check_other_lane` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<safety_check_ns>enable_safety_check` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<safety_check_ns>hysteresis_factor_expand_rate` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<safety_check_ns>backward_path_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<safety_check_ns>forward_path_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<safety_check_ns>publish_debug_marker` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<safety_check_ns>collision_check_yaw_diff_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<rss_ns>rear_vehicle_reaction_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<rss_ns>rear_vehicle_safety_time_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<rss_ns>lateral_distance_max_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<rss_ns>longitudinal_distance_min_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<rss_ns>longitudinal_velocity_delta_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `<rss_ns>extended_polygon_policy` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.surround_moving_obstacle_check.search_radius` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.surround_moving_obstacle_check.th_moving_obstacle_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |
| `start_planner.debug.print_debug_info` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/data_structs.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `start_planner.th_arrived_distance` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.th_stopped_velocity` | `0.01` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.th_stopped_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.collision_check_margins` | `[2.0, 1.0, 0.5, 0.1]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.collision_check_margin_from_front_object` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.skip_rear_vehicle_check` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.extra_width_margin_for_rear_obstacle` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.th_moving_object_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.object_types_to_check_for_path_generation.check_unknown` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.th_distance_to_middle_of_the_road` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.center_line_path_interval` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.check_shift_path_lane_departure` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.allow_check_shift_path_lane_departure_override` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.shift_collision_check_distance_from_end` | `-10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.minimum_shift_pull_out_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.minimum_shift_length` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.maximum_longitudinal_deviation` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.lateral_jerk` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.lateral_acceleration_sampling_num` | `3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.minimum_lateral_acc` | `0.15` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.maximum_lateral_acc` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.maximum_curvature` | `0.07` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.end_pose_curvature_threshold` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.geometric_collision_check_distance_from_end` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.arc_path_interval` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.divide_pull_out_path` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.geometric_pull_out_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.lane_departure_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.lane_departure_check_expansion_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.backward_velocity` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.geometric_pull_out_max_steer_angle_margin_scale` | `0.72` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.clothoid_initial_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.clothoid_acceleration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.clothoid_max_steer_angles_deg` | `[5.0, 10.0, 20.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.clothoid_max_steer_angle_rate_deg_per_sec` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.clothoid_collision_check_distance_from_end` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.check_clothoid_path_lane_departure` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.enable_back` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.search_priority` | `["SHIFT", "GEOMETRIC"]` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.search_policy` | `"planner_priority"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.max_back_distance` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.backward_search_resolution` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.backward_path_update_duration` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.ignore_distance_from_lane_end` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.prepare_time_before_start` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.turn_signal_on_centerline_start` | `"RIGHT"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.enable_freespace_planner` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.end_pose_search_start_distance` | `20.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.end_pose_search_end_distance` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.end_pose_search_interval` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.freespace_planner_algorithm` | `"astar"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.vehicle_shape_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.time_limit` | `3000.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.max_turning_ratio` | `0.7` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.turning_steps` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.theta_size` | `120` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.angle_goal_range` | `6.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.lateral_goal_range` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.longitudinal_goal_range` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.curve_weight` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.reverse_weight` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.search_configs.direction_change_weight` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.costmap_configs.obstacle_threshold` | `30` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.search_method` | `"forward"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.only_behind_solutions` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.use_back` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.adapt_expansion_distance` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.expansion_distance` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.near_goal_distance` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.distance_heuristic_weight` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.smoothness_weight` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.obstacle_distance_weight` | `1.75` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.astar.goal_lat_distance_weight` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.rrtstar.enable_update` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.rrtstar.use_informed_sampling` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.rrtstar.max_planning_time` | `150.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.rrtstar.neighbor_radius` | `8.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.freespace_planner.rrtstar.margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.stop_condition.maximum_deceleration_for_stop` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.stop_condition.maximum_jerk_for_stop` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.ego_predicted_path.min_velocity` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.ego_predicted_path.min_acceleration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.ego_predicted_path.time_horizon_for_front_object` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.ego_predicted_path.time_horizon_for_rear_object` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.ego_predicted_path.time_resolution` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.ego_predicted_path.delay_until_departure` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.safety_check_time_horizon` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.safety_check_time_resolution` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_check_forward_distance` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_check_backward_distance` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.ignore_object_velocity_threshold` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_types_to_check.check_unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_lane_configuration.check_current_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_lane_configuration.check_right_side_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_lane_configuration.check_left_side_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_lane_configuration.check_shoulder_lane` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.object_lane_configuration.check_other_lane` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.include_opposite_lane` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.invert_opposite_lane` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.check_all_predicted_path` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.use_all_predicted_path` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.target_filtering.use_predicted_path_outside_lanelet` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.enable_safety_check` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.publish_debug_marker` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.rss_params.rear_vehicle_reaction_time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.rss_params.rear_vehicle_safety_time_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.rss_params.lateral_distance_max_threshold` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.rss_params.longitudinal_distance_min_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.rss_params.longitudinal_velocity_delta_time` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.rss_params.extended_polygon_policy` | `"along_path"` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.hysteresis_factor_expand_rate` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.collision_check_yaw_diff_threshold` | `1.578` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.backward_path_length` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.path_safety_check.safety_check_params.forward_path_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.search_radius` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.th_moving_obstacle_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_bicycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_motorcycle` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_pedestrian` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.surround_moving_obstacle_check.object_types_to_check.check_unknown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |
| `start_planner.debug.print_debug_info` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/config/start_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_cmake`, `autoware_freespace_planning_algorithms`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_pyplot`, `autoware_rtc_interface`, `autoware_test_utils`, `autoware_utils`, `eigen3_cmake_module`, `pluginlib`, `pybind11_vendor`, `rclcpp`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 121 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 17 |
| `pluginlib` | 3 |
| `parameters` | 134 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/README.md`
