> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_behavior_path_avoidance_by_lane_change_module

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module`
- Version: `0.51.0`
- Package description: The behavior_path_avoidance_by_lane_change_module package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager -> autoware::behavior_path_planner::SceneModuleManagerInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |

## Purpose And Role

This is a sub-module to avoid obstacles by lane change maneuver. This module is designed as one of the obstacle avoidance features and generates a lane change path if the following conditions are satisfied. - Exist lane changeable lanelet. - Exist avoidance target objects on ego driving lane. Basically, this module is implemented by reusing the avoidance target filtering logic of the existing Static Object Avoidance Module and the path generation logic of the Normal Lane Change Module. On the other hand, the conditions under which the module is activated differ fr

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
| `avoidance_by_lane_change.execute_object_longitudinal_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance_by_lane_change.execute_only_when_lane_change_finish_before_object` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.resample_interval_for_planning` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.resample_interval_for_output` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.th_moving_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.th_moving_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.max_expand_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.envelope_buffer_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.lateral_margin.soft_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.lateral_margin.hard_margin` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.lateral_margin.hard_margin_for_parked_vehicle` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance_by_lane_change.target_object.lower_distance_for_polygon_expansion` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance_by_lane_change.target_object.upper_distance_for_polygon_expansion` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance_by_lane_change.target_object.` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.object_check_goal_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.max_compensation_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.parked_vehicle.th_offset_from_centerline` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.parked_vehicle.th_shiftable_ratio` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.parked_vehicle.min_road_shoulder_width` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.policy` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.wait_and_see.target_behaviors` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.wait_and_see.th_closest_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.condition.th_stopped_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.condition.th_moving_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.ignore_area.traffic_light.front_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.ignore_area.crosswalk.front_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.avoidance_for_ambiguous_vehicle.ignore_area.crosswalk.behind_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.avoidance.longitudinal.min_prepare_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.avoidance.longitudinal.max_prepare_time` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.avoidance.longitudinal.min_prepare_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.avoidance.longitudinal.min_slow_down_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.avoidance.longitudinal.buf_slow_down_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.avoidance.longitudinal.nominal_avoidance_speed` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.detection_area.static` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.detection_area.min_forward_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.detection_area.max_forward_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.target_filtering.detection_area.backward_distance` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |
| `avoidance.safety_check.hysteresis_factor_expand_rate` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `avoidance_by_lane_change.execute_object_longitudinal_margin` | `80.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.execute_only_when_lane_change_finish_before_object` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.th_moving_speed` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.envelope_buffer_margin` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.lateral_margin.hard_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.car.lateral_margin.hard_margin_for_parked_vehicle` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.th_moving_speed` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.envelope_buffer_margin` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.lateral_margin.hard_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.truck.lateral_margin.hard_margin_for_parked_vehicle` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.th_moving_speed` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.envelope_buffer_margin` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.lateral_margin.hard_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bus.lateral_margin.hard_margin_for_parked_vehicle` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.th_moving_speed` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.envelope_buffer_margin` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.lateral_margin.hard_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.trailer.lateral_margin.hard_margin_for_parked_vehicle` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.th_moving_speed` | `0.28` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.envelope_buffer_margin` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.lateral_margin.hard_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.unknown.lateral_margin.hard_margin_for_parked_vehicle` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.th_moving_speed` | `0.28` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.envelope_buffer_margin` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.lateral_margin.hard_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.bicycle.lateral_margin.hard_margin_for_parked_vehicle` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.th_moving_speed` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.envelope_buffer_margin` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.lateral_margin.hard_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.motorcycle.lateral_margin.hard_margin_for_parked_vehicle` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.th_moving_speed` | `0.28` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.th_moving_time` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.max_expand_ratio` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.envelope_buffer_margin` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.lateral_margin.soft_margin` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.lateral_margin.hard_margin` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.pedestrian.lateral_margin.hard_margin_for_parked_vehicle` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.lower_distance_for_polygon_expansion` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_object.upper_distance_for_polygon_expansion` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.car` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.truck` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.bus` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.trailer` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.unknown` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.bicycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.motorcycle` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |
| `avoidance_by_lane_change.target_filtering.target_type.pedestrian` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_behavior_path_lane_change_module`, `autoware_behavior_path_planner`, `autoware_behavior_path_planner_common`, `autoware_behavior_path_static_obstacle_avoidance_module`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_rtc_interface`, `autoware_utils`, `eigen3_cmake_module`, `pluginlib`, `rclcpp`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 16 |
| `lifecycle` | 0 |
| `component` | 0 |
| `tf2` | 0 |
| `pluginlib` | 3 |
| `parameters` | 39 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md`
