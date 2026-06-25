---
source_type: rendered_autoware_docs
site: autoware_core
title: Stop Line
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/
rendered_html: rendered/autoware_core_html/index-66.html
generated: 2026-06-24T15:00:05.076328+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Stop Line

## Role

This module plans the vehicle's velocity to ensure it stops just before stop lines and can resume movement after stopping.

![stop line](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/stop_line.svg)

## Activation Timing

This module is activated when there is a stop line in a target lane.

## Module Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `stop_margin` | double | Margin that the vehicle tries to stop in before stop\_line |
| `required_stop_duration_sec` | double | [s] Required stop duration at the stop line. The vehicle transitions from STOPPED to START state after stopping for this duration. |
| `vehicle_stopped_duration_threshold` | double | [s] Duration threshold for determining if the vehicle is stopped. Used to check if the vehicle has been stopped for at least this duration. |
| `hold_stop_margin_distance` | double | [m] Parameter for restart prevention (See Algorithm section). Also, when the ego vehicle is within this distance from a stop line, the ego state becomes STOPPED from APPROACHING |
| `use_initialization_stop_state` | bool | Flag to determine whether to return to the approaching state when the vehicle moves away from a stop line. |

## Inner-workings / Algorithms

- Gets a stop line from map information.
- Inserts a stop point on the path from the stop line defined in the map and the ego vehicle length.
- Sets velocities of the path after the stop point to 0[m/s].
- Releases the inserted stop velocity when the vehicle halts at the stop point for `required_stop_duration_sec` seconds.

### Flowchart

![uml diagram](../../../../../assets/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/image-01-3df3c4ba05.svg)

This algorithm is based on `segment`.
`segment` consists of two node points. It's useful for removing boundary conditions because if `segment(i)` exists we can assume `node(i)` and `node(i+1)` exist.

![node_and_segment](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/node_and_segment.drawio.svg)

First, this algorithm finds a collision between reference path and stop line.
Then, we can get `collision segment` and `collision point`.

![find_collision_segment](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/find_collision_segment.drawio.svg)

Next, based on `collision point`, it finds `offset segment` by iterating backward points up to a specific offset length.
The offset length is `stop_margin`(parameter) + `base_link to front`(to adjust head pose to stop line).
Then, we can get `offset segment` and `offset from segment start`.

![find_offset_segment](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/find_offset_segment.drawio.svg)

After that, we can calculate an offset point from `offset segment` and `offset`. This will be `stop_pose`.

![calculate_stop_pose](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/calculate_stop_pose.drawio.svg)

### Restart Prevention

If the vehicle requires X meters (e.g. 0.5 meters) to stop once it starts moving due to poor vehicle control performance, it may overshoot the stopping position, which must be strictly observed. This happens when the vehicle begins to move in order to approach a nearby stop point (e.g. 0.3 meters away).

This module includes the parameter `hold_stop_margin_distance` to prevent redundant restarts in such a situation. If the vehicle is stopped within `hold_stop_margin_distance` meters of the stop point (\_front\_to\_stop\_line < hold\_stop\_margin\_distance), the module determines that the vehicle has already stopped at the stop point and will maintain the current stopping position, even if the vehicle has come to a stop due to other factors.

![example](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/restart_prevention.svg)

parameters


![example](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/restart.svg)

outside the hold\_stop\_margin\_distance


![example](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/docs/keep_stopping.svg)

inside the hold\_stop\_margin\_distance
