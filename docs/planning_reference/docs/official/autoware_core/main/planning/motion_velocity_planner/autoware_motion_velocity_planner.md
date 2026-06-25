---
source_type: rendered_autoware_docs
site: autoware_core
title: Motion Velocity Planner
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_planner/
rendered_html: rendered/autoware_core_html/index-68.html
generated: 2026-06-24T15:00:05.113927+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Motion Velocity Planner

## Overview

`motion_velocity_planner` is a planner to adjust the trajectory velocity based on the obstacles around the vehicle.
It loads modules as plugins. Please refer to the links listed below for detail on each module.

![Architecture](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_planner/docs/MotionVelocityPlanner-InternalInterface.drawio.svg)

- [Out of Lane](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/README.md)

Each module calculates stop and slow down points to be inserted in the ego trajectory.
These points are assumed to correspond to the `base_link` frame of the ego vehicle as it follows the trajectory.
This means that to stop before a wall, a stop point is inserted in the trajectory at a distance ahead of the wall equal to the vehicle front offset (wheelbase + front overhang, see the [vehicle dimensions](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/components/vehicle-dimensions/).

![set_stop_velocity](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_planner/docs/set_stop_velocity.drawio.svg)

## Input topics

| Name | Type | Description |
| --- | --- | --- |
| `~/input/trajectory` | autoware\_planning\_msgs::msg::Trajectory | input trajectory |
| `~/input/vector_map` | autoware\_map\_msgs::msg::LaneletMapBin | vector map |
| `~/input/vehicle_odometry` | nav\_msgs::msg::Odometry | vehicle position and velocity |
| `~/input/accel` | geometry\_msgs::msg::AccelWithCovarianceStamped | vehicle acceleration |
| `~/input/dynamic_objects` | autoware\_perception\_msgs::msg::PredictedObjects | dynamic objects |
| `~/input/no_ground_pointcloud` | sensor\_msgs::msg::PointCloud2 | obstacle pointcloud |
| `~/input/traffic_signals` | autoware\_perception\_msgs::msg::TrafficLightGroupArray | traffic light states |
| `~/input/occupancy_grid` | nav\_msgs::msg::OccupancyGrid | occupancy grid |

## Output topics

| Name | Type | Description |
| --- | --- | --- |
| `~/output/trajectory` | autoware\_planning\_msgs::msg::Trajectory | Ego trajectory with updated velocity profile |
| `~/output/planning_factors/<MODULE_NAME>` | autoware\_internal\_planning\_msgs::msg::PlanningFactorsArray | factors causing change in the ego velocity profile |

## Services

| Name | Type | Description |
| --- | --- | --- |
| `~/service/load_plugin` | autoware\_motion\_velocity\_planner::srv::LoadPlugin | To request loading a plugin |
| `~/service/unload_plugin` | autoware\_motion\_velocity\_planner::srv::UnloadPlugin | To request unloaded a plugin |

## Node parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `launch_modules` | vector\<string> | module names to launch |

In addition, the following parameters should be provided to the node:

- [nearest search parameters](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml);
- [vehicle info parameters](https://github.com/autowarefoundation/autoware_core/blob/main/description/autoware_sample_vehicle_description/config/vehicle_info.param.yaml);
- [common planning parameters](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml);
- [smoother parameters](https://github.com/autowarefoundation/autoware_core/tree/main/planning/autoware_velocity_smoother/#parameters)
- Parameters of each plugin that will be loaded.
