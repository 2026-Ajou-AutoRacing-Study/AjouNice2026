> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_planner_common

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common`
- Version: `0.51.0`
- Package description: The autoware_behavior_path_planner_common package
- Primary README: `not found`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| _none detected_ |  |  |

## Purpose And Role

No concise README paragraph was detected. Use the source files below as primary evidence.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/info/` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `~/debug/` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `~/virtual_wall/` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `~/drivable_lanes/` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `~/debug/processing_time_detail_ms/` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `traffic_light_signal_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `backward_path_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `forward_path_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `normal.min_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `normal.max_acc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `max_vel` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `backward_length_buffer_for_end_of_pull_over` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `backward_length_buffer_for_end_of_pull_out` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `minimum_pull_over_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `refine_goal_search_radius_range` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_intersection_search_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_intersection_angle_threshold_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_minimum_search_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_search_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_shift_length_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_remaining_shift_length_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_remaining_distance_to_bound_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_on_swerving` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_roundabout_on_entry` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_roundabout_on_exit` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_roundabout_entry_indicator_persistence` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_roundabout_search_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_roundabout_angle_threshold_deg` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_roundabout_backward_depth` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `turn_signal_path_backward_length` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `enable_akima_spline_first` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `enable_cog_on_centerline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `input_path_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `output_path_interval` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `ego_nearest_dist_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `ego_nearest_yaw_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp` |
| `enable_all_modules_auto_mode` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `<ns>enable_rtc` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `<ns>enable_simultaneous_execution_as_approved_module` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |
| `<ns>enable_simultaneous_execution_as_candidate_module` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/interface/scene_module_manager_interface.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| _none detected_ |  |  |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_adapi_v1_msgs`, `autoware_cmake`, `autoware_deprecated_boundary_departure_checker`, `autoware_freespace_planning_algorithms`, `autoware_internal_planning_msgs`, `autoware_interpolation`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_objects_of_interest_marker_interface`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_rtc_interface`, `autoware_test_utils`, `autoware_traffic_light_utils`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `magic_enum`, `rclcpp`, `tf2`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 82 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 36 |
| `pluginlib` | 0 |
| `parameters` | 62 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_drivable_area_design.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_path_generation_design.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_safety_check.md`
- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_turn_signal_design.md`
