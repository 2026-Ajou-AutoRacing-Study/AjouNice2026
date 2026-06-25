---
source_type: rendered_autoware_docs
site: autoware_core
title: Behavior Velocity Planner
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/
rendered_html: rendered/autoware_core_html/index-64.html
generated: 2026-06-24T15:00:05.041826+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Behavior Velocity Planner

## Overview

`behavior_velocity_planner` is a planner that adjust velocity based on the traffic rules.
It loads modules as plugins. Please refer to the links listed below for detail on each module.

![Architecture](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/docs/BehaviorVelocityPlanner-Architecture.drawio.svg)

- [Blind Spot](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/)
- [Crosswalk](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/)
- [Walkway](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module/)
- [Detection Area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/)
- [Intersection](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [MergeFromPrivate](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/#merge-from-private)
- [Stop Line](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/)
- [Virtual Traffic Light](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module/)
- [Traffic Light](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/)
- [Occlusion Spot](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/)
- [No Stopping Area](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/)
- [Speed Bump](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/)

When each module plans velocity, it considers based on `base_link`(center of rear-wheel axis) pose.
So for example, in order to stop at a stop line with the vehicles' front on the stop line, it calculates `base_link` position from the distance between `base_link` to front and modifies path velocity from the `base_link` position.

![set_stop_velocity](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/docs/set_stop_velocity.drawio.svg)

## Input topics

| Name | Type | Description |
| --- | --- | --- |
| `~input/path_with_lane_id` | autoware\_internal\_planning\_msgs::msg::PathWithLaneId | path with lane\_id |
| `~input/vector_map` | autoware\_map\_msgs::msg::LaneletMapBin | vector map |
| `~input/vehicle_odometry` | nav\_msgs::msg::Odometry | vehicle velocity |
| `~input/dynamic_objects` | autoware\_perception\_msgs::msg::PredictedObjects | dynamic objects |
| `~input/no_ground_pointcloud` | sensor\_msgs::msg::PointCloud2 | obstacle pointcloud |
| `~/input/compare_map_filtered_pointcloud` | sensor\_msgs::msg::PointCloud2 | obstacle pointcloud filtered by compare map. Note that this is used only when the detection method of run out module is Points. |
| `~input/traffic_signals` | autoware\_perception\_msgs::msg::TrafficLightGroupArray | traffic light states |

## Output topics

| Name | Type | Description |
| --- | --- | --- |
| `~output/path` | autoware\_planning\_msgs::msg::Path | path to be followed |
| `~output/stop_reasons` | tier4\_planning\_msgs::msg::StopReasonArray | reasons that cause the vehicle to stop |

## Node parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `launch_modules` | vector<string> | module names to launch |
| `forward_path_length` | double | forward path length |
| `backward_path_length` | double | backward path length |
| `max_accel` | double | (to be a global parameter) max acceleration of the vehicle |
| `system_delay` | double | (to be a global parameter) delay time until output control command |
| `delay_response_time` | double | (to be a global parameter) delay time of the vehicle's response to control commands |

## Traffic Light Handling in sim/real

The handling of traffic light information varies depending on the usage. In the below table, the traffic signal topic element for the corresponding lane is denoted as `info`, and if `info` is not available, it is denoted as `null`.

| module \ case | `info` is `null` | `info` is not `null` |
| --- | --- | --- |
| intersection\_occlusion(`is_simulation = *`)  - `info` is the latest non-`null` information | GO(occlusion is ignored) | intersection\_occlusion uses the latest non UNKNOWN observation in the queue up to present.  - If `info` is `GREEN or UNKNOWN`, occlusion is cared - If `info` is `RED or YELLOW`, occlusion is ignored(GO) - NOTE: Currently timeout is not considered |
| traffic\_light(sim, `is_simulation = true`)  - `info` is current information | GO | traffic\_light uses the perceived traffic light information at present directly.  - If `info` is timeout, STOP whatever the color is - If `info` is not timeout, then act according to the color. If `info` is `UNKNOWN`, STOP |
| traffic\_light(real, `is_simulation = false`)  - `info` is current information | STOP | ⁠ |
| crosswalk with Traffic Light(`is_simulation = *`)  - `info` is current information | default | - If `disable_yield_for_new_stopped_object` is true, each sub scene\_module ignore newly detected pedestrians after module instantiation. - If `ignore_with_traffic_light` is true, occlusion detection is skipped. |
| map\_based\_prediction(`is_simulation = *`)  - `info` is current information | default | If a pedestrian traffic light is  - RED, surrounding pedestrians are not predicted. - GREEN, stopped pedestrians are not predicted. |
