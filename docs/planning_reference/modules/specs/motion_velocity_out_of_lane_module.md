> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_motion_velocity_out_of_lane_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module`
- Version: `0.51.0`
- Package description: The motion_velocity_out_of_lane_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::motion_velocity_planner::OutOfLaneModule -> autoware::motion_velocity_planner::PluginModuleInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |

## Purpose And Role

There are cases where the ego vehicle footprint goes out of the driving lane, for example when taking a narrow turn with a large vehicle. The `out_of_lane` module adds deceleration and stop points to the ego trajectory in order to prevent collisions from occurring in these out of lane cases. This module is activated if the launch parameter `launch_out_of_lane_module` is set to true. This module calculates if out of lane collisions occur and insert stop point before the collisions if necessary.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `~/` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `~/debug/` | `autoware_planning_msgs::msg::Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `<ns_>.mode` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.skip_if_already_overlapping` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.max_arc_length` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.threshold.time_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.ttc.threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.ttc.release_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.objects.minimum_velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.objects.predicted_path_min_confidence` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.objects.cut_predicted_paths_beyond_red_lights` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.objects.ignore_behind_ego` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.objects.validate_predicted_paths_on_lanelets` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.objects.extra_width` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.precision` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.use_map_stop_lines` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.min_on_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.min_off_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.update_distance_th` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.longitudinal_distance_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.lateral_distance_buffer` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.slowdown.velocity` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.action.stop.distance_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.ego.extra_front_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.ego.extra_rear_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.ego.extra_left_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |
| `<ns_>.ego.extra_right_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/out_of_lane_module.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `out_of_lane.mode` | `ttc` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.skip_if_already_overlapping` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.max_arc_length` | `100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.threshold.time_threshold` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.ttc.threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.ttc.release_threshold` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.objects.minimum_velocity` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.objects.predicted_path_min_confidence` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.objects.cut_predicted_paths_beyond_red_lights` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.objects.ignore_behind_ego` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.objects.validate_predicted_paths_on_lanelets` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.objects.extra_width` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.use_map_stop_lines` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.precision` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.longitudinal_distance_buffer` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.lateral_distance_buffer` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.min_on_duration` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.min_off_duration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.update_distance_th` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.slowdown.distance_threshold` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.slowdown.velocity` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.action.stop.distance_threshold` | `15.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.ego.extra_front_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.ego.extra_rear_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.ego.extra_right_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |
| `out_of_lane.ego.extra_left_offset` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/config/out_of_lane.param.yaml` |

## Dependencies

`ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_motion_velocity_planner_common`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_traffic_light_utils`, `autoware_utils`, `autoware_vehicle_info_utils`, `geometry_msgs`, `libboost-dev`, `pluginlib`, `rclcpp`, `tf2`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 18 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 5 |
| `pluginlib` | 3 |
| `parameters` | 26 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/README.md`
