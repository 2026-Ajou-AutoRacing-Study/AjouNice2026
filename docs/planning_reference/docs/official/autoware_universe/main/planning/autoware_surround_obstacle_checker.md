---
source_type: rendered_autoware_docs
site: autoware_universe
title: Surround Obstacle Checker
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_surround_obstacle_checker/
rendered_html: rendered/autoware_universe_html/index-170.html
generated: 2026-06-24T14:59:32.573087+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Surround Obstacle Checker

## Purpose

This module subscribes required data (ego-pose, obstacles, etc), and publishes zero velocity limit to keep stopping if any of stop conditions are satisfied.

## Inner-workings / Algorithms

### Flow chart

![uml diagram](../../../../assets/autoware_universe/main/planning/autoware_surround_obstacle_checker/image-01-1820309145.svg)

![](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_surround_obstacle_checker/media/check_distance.drawio.svg)

### Algorithms

### Check data

Check that `surround_obstacle_checker` receives no ground pointcloud, dynamic objects and current velocity data.

### Get distance to nearest object

Calculate distance between ego vehicle and the nearest object.
In this function, it calculates the minimum distance between the polygon of ego vehicle and all points in pointclouds and the polygons of dynamic objects.

### Stop requirement

If it satisfies all following conditions, it plans stopping.

- Ego vehicle is stopped
- It satisfies any following conditions
  1. The distance to nearest obstacle satisfies following conditions
     - If state is `State::PASS`, the distance is less than `surround_check_distance`
     - If state is `State::STOP`, the distance is less than `surround_check_recover_distance`
  2. If it does not satisfies the condition in 1, elapsed time from the time it satisfies the condition in 1 is less than `state_clear_time`

### States

To prevent chattering, `surround_obstacle_checker` manages two states.
As mentioned in stop condition section, it prevents chattering by changing threshold to find surround obstacle depending on the states.

- `State::PASS` : Stop planning is released
- `State::STOP` ：While stop planning

## Inputs / Outputs

### Input

| Name | Type | Description |
| --- | --- | --- |
| `/perception/obstacle_segmentation/pointcloud` | `sensor_msgs::msg::PointCloud2` | Pointcloud of obstacles which the ego-vehicle should stop or avoid |
| `/perception/object_recognition/objects` | `autoware_perception_msgs::msg::PredictedObjects` | Dynamic objects |
| `/localization/kinematic_state` | `nav_msgs::msg::Odometry` | Current twist |
| `/tf` | `tf2_msgs::msg::TFMessage` | TF |
| `/tf_static` | `tf2_msgs::msg::TFMessage` | TF static |

### Output

| Name | Type | Description |
| --- | --- | --- |
| `~/output/velocity_limit_clear_command` | `autoware_internal_planning_msgs::msg::VelocityLimitClearCommand` | Velocity limit clear command |
| `~/output/max_velocity` | `autoware_internal_planning_msgs::msg::VelocityLimit` | Velocity limit command |
| `~/output/no_start_reason` | `diagnostic_msgs::msg::DiagnosticStatus` | No start reason |
| `~/debug/marker` | `visualization_msgs::msg::MarkerArray` | Marker for visualization |
| `~/debug/footprint` | `geometry_msgs::msg::PolygonStamped` | Ego vehicle base footprint for visualization |
| `~/debug/footprint_offset` | `geometry_msgs::msg::PolygonStamped` | Ego vehicle footprint with `surround_check_distance` offset for visualization |
| `~/debug/footprint_recover_offset` | `geometry_msgs::msg::PolygonStamped` | Ego vehicle footprint with `surround_check_recover_distance` offset for visualization |

## Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| pointcloud.enable\_check | boolean | enable to check surrounding pointcloud | false | N/A |
| pointcloud.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| pointcloud.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| pointcloud.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| unknown.enable\_check | boolean | enable to check surrounding unknown objects | true | N/A |
| unknown.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| unknown.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| unknown.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| car.enable\_check | boolean | enable to check surrounding car | true | N/A |
| car.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| car.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| car.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| truck.enable\_check | boolean | enable to check surrounding truck | true | N/A |
| truck.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| truck.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| truck.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| bus.enable\_check | boolean | enable to check surrounding bus | true | N/A |
| bus.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| bus.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| bus.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| trailer.enable\_check | boolean | enable to check surrounding trailer | true | N/A |
| trailer.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| trailer.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| trailer.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| motorcycle.enable\_check | boolean | enable to check surrounding motorcycle | true | N/A |
| motorcycle.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| motorcycle.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| motorcycle.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| bicycle.enable\_check | boolean | enable to check surrounding bicycle | true | N/A |
| bicycle.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| bicycle.surround\_check\_side\_distance | float | f objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| bicycle.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| pedestrian.enable\_check | boolean | enable to check surrounding pedestrian | true | N/A |
| pedestrian.surround\_check\_front\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| pedestrian.surround\_check\_side\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| pedestrian.surround\_check\_back\_distance | float | If objects exist in this distance, transit to "exist-surrounding-obstacle" status. [m] | 0.5 | ≥0.0 |
| surround\_check\_hysteresis\_distance | float | If no object exists in this hysteresis distance added to the above distance, transit to "non-surrounding-obstacle" status [m] | 0.3 | ≥0.0 |
| state\_clear\_time | float | Threshold to clear stop state [s] | 2.0 | ≥0.0 |
| stop\_state\_ego\_speed | float | Threshold to check ego vehicle stopped [m/s] | 0.1 | ≥0.0 |
| publish\_debug\_footprints | boolean | Publish vehicle footprint & footprints with surround\_check\_distance and surround\_check\_recover\_distance offsets. | true | N/A |
| debug\_footprint\_label | string | select the label for debug footprint | car | ['pointcloud', 'unknown', 'car', 'truck', 'bus', 'trailer', 'motorcycle', 'bicycle', 'pedestrian'] |

| Name | Type | Description | Default value |
| --- | --- | --- | --- |
| `enable_check` | `bool` | Indicates whether each object is considered in the obstacle check target. | `true` for objects; `false` for point clouds |
| `surround_check_front_distance` | `bool` | If there are objects or point clouds within this distance in front, transition to the "exist-surrounding-obstacle" status [m]. | 0.5 |
| `surround_check_side_distance` | `double` | If there are objects or point clouds within this side distance, transition to the "exist-surrounding-obstacle" status [m]. | 0.5 |
| `surround_check_back_distance` | `double` | If there are objects or point clouds within this back distance, transition to the "exist-surrounding-obstacle" status [m]. | 0.5 |
| `surround_check_hysteresis_distance` | `double` | If no object exists within `surround_check_xxx_distance` plus this additional distance, transition to the "non-surrounding-obstacle" status [m]. | 0.3 |
| `state_clear_time` | `double` | Threshold to clear stop state [s] | 2.0 |
| `stop_state_ego_speed` | `double` | Threshold to check ego vehicle stopped [m/s] | 0.1 |
| `stop_state_entry_duration_time` | `double` | Threshold to check ego vehicle stopped [s] | 0.1 |
| `publish_debug_footprints` | `bool` | Publish vehicle footprint with/without offsets | `true` |

## Assumptions / Known limits

To perform stop planning, it is necessary to get obstacle pointclouds data.
Hence, it does not plan stopping if the obstacle is in blind spot.
