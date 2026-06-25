> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_run_out_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module`
- Version: `0.51.0`
- Package description: run out module for the motion_velocity_planner
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::RunOutModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/run_out_module.cpp` |

## Purpose And Role

The `run_out` module adds deceleration and stop points to the ego trajectory in order to prevent collisions with objects that are moving towards the ego vehicle path. This module is activated if the launch parameter `launch_run_out_module` is set to true. This module calculates the times when the ego vehicle and the objects are predicted to overlap each other's trajectories. These times are then used to decide whether to stop before the overlap or not.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/run_out_module.cpp` |
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/run_out_module.cpp` |
| `~/debug/` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/run_out_module.cpp` |
| `~/` | `autoware::universe_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/run_out_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `<const std::string & ns>.{<object_label>|default}.<const std::string & param>` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore.if_on_ego_trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore.if_behind_ego` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore.polygon_types` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore.lanelet_subtypes` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore_collisions.polygon_types` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore_collisions.lanelet_subtypes` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore_collisions.start_ignore_time` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore_collisions.start_ignore_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore.if_stopped` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..ignore.stopped_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..confidence_filtering.threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..confidence_filtering.only_use_highest` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.polygon_types` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.lanelet_subtypes` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.linestring_types` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.strict_polygon_types` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.strict_lanelet_subtypes` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.strict_linestring_types` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..cut_predicted_paths.if_crossing_ego_from_behind` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..preserved_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..preserved_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |
| `<ns>.{<object_label>|default}..standstill_duration_after_cut` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/src/parameters.hpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `run_out.collision.time_margin` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.time_overlap_tolerance` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.same_direction_angle_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.opposite_direction_angle_threshold` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.ignore_conditions.if_ego_arrives_first.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.ignore_conditions.if_ego_arrives_first.margin.ego_enter_times` | `[0.0, 3.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.ignore_conditions.if_ego_arrives_first.margin.time_margins` | `[0.0, 6.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.ignore_conditions.if_ego_arrives_first.max_overlap_duration` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.ignore_conditions.if_ego_arrives_first_and_cannot_stop.enable` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.collision.ignore_conditions.if_ego_arrives_first_and_cannot_stop.deceleration_limit` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.slowdown.on_time_buffer` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.slowdown.off_time_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.slowdown.distance_buffer` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.slowdown.deceleration_limit` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.on_time_buffer` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.off_time_buffer` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.distance_buffer` | `2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.deceleration_limit` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.keep_condition.time` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.keep_condition.distance` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.stop.reuse_margin` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.ego.lateral_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.ego.longitudinal_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.target_labels` | `["PEDESTRIAN", "BICYCLE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore.if_stopped` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore.stopped_velocity_threshold` | `0.25` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore.if_on_ego_trajectory` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore.if_behind_ego` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore.polygon_types` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore.lanelet_subtypes` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore_collisions.polygon_types` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore_collisions.lanelet_subtypes` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore_collisions.start_ignore_time` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.ignore_collisions.start_ignore_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.confidence_filtering.threshold` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.confidence_filtering.only_use_highest` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.if_crossing_ego_from_behind` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.polygon_types` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.linestring_types` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.lanelet_subtypes` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.strict_polygon_types` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.strict_linestring_types` | `["guard_rail"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.cut_predicted_paths.strict_lanelet_subtypes` | `["NONE"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.preserved_duration` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.preserved_distance` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.DEFAULT.standstill_duration_after_cut` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.PEDESTRIAN.ignore.if_stopped` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.PEDESTRIAN.ignore.lanelet_subtypes` | `["crosswalk", "walkway"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.PEDESTRIAN.cut_predicted_paths.lanelet_subtypes` | `["crosswalk", "walkway"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.PEDESTRIAN.preserved_duration` | `2.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.PEDESTRIAN.preserved_distance` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.objects.BICYCLE.ignore_collisions.lanelet_subtypes` | `["crosswalk", "walkway"]` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.debug.object_label` | `"PEDESTRIAN"` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.debug.enabled_markers.ego_footprint` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.debug.enabled_markers.objects` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.debug.enabled_markers.collisions` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.debug.enabled_markers.decisions` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |
| `run_out.debug.enabled_markers.filtering_data` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/config/run_out.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_universe_utils`, `autoware_velocity_smoother`, `diagnostic_updater`, `eigen`, `eigen3_cmake_module`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `rclcpp_components`, `rosidl_default_generators`, `rosidl_default_runtime`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 55 |
| `lifecycle` | 0 |
| `component` | 1 |
| `tf2` | 5 |
| `pluginlib` | 3 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/README.md`
