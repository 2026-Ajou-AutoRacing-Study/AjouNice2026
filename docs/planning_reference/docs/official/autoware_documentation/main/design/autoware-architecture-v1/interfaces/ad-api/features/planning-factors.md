---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Planning factors
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/planning-factors/
rendered_html: rendered/autoware_documentation_html/index-115.html
generated: 2026-06-24T14:59:48.295969+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning factors

## Related API

- [/api/planning/velocity\_factors](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/planning/velocity_factors/)
- [/api/planning/steering\_factors](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/planning/steering_factors/)

## Description

This API manages the planned behavior of the vehicle.
Applications can notify the vehicle behavior to the people around and visualize it for operator and passengers.

## Velocity factors

The velocity factors is an array of information on the behavior that the vehicle stops or slows down.
Each factor has a behavior type which is described below.
Some behavior types have sequence and details as additional information.

| Behavior | Description |
| --- | --- |
| surrounding-obstacle | There are obstacles immediately around the vehicle. |
| route-obstacle | There are obstacles along the route ahead. |
| intersection | There are obstacles in other lanes in the path. |
| crosswalk | There are obstacles on the crosswalk. |
| rear-check | There are obstacles behind that would be in a human driver's blind spot. |
| user-defined-attention-area | There are obstacles in the predefined attention area. |
| roundabout | There are obstacles in the roundabout. |
| no-stopping-area | There is not enough space beyond the no stopping area. |
| stop-sign | A stop by a stop sign. |
| traffic-signal | A stop by a traffic signal. |
| v2x-gate-area | A stop by a gate area. It has enter and leave as sequences and v2x type as details. |
| merge | A stop before merging lanes. |
| sidewalk | A stop before crossing the sidewalk. |
| lane-change | A lane change. |
| avoidance | A path change to avoid an obstacle in the current lane. |
| emergency-operation | A stop by emergency instruction from the operator. |

Each factor also provides status, poses in the base link frame, and distance from that pose.
As the vehicle approaches the stop position, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position and stops, the status will be STOPPED.
The pose indicates the stop position, or the base link if the stop position cannot be calculated.

![velocity-factors](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/planning-factors/velocity-factors.drawio.svg)

## Steering factors

The steering factors is an array of information on the maneuver that requires use of turn indicators, such as turning left or right.
Each factor has a behavior type which is described below and steering direction.
Some behavior types have sequence and details as additional information.

| Behavior | Description |
| --- | --- |
| intersection | A turning left or right at an intersection. |
| lane-change | A lane change. |
| avoidance | A path change to avoid an obstacle. It has a sequence of change and return. |
| start-planner | T.B.D. |
| goal-planner | T.B.D. |
| emergency-operation | A path change by emergency instruction from the operator. |

Each factor also provides status, poses in the base link frame, and distances from that poses.
As the vehicle approaches the position to start steering, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position, the status will be TURNING.
The poses indicate the start and end position of the section where the status is TURNING.

![steering-factors-1](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/planning-factors/steering-factors-1.drawio.svg)

In cases such as lane change and avoidance, the vehicle will start steering at any position in the range depending on the situation.
For these types, the section where the status is TURNING will be updated dynamically and the poses will follow that.

![steering-factors-2](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/planning-factors/steering-factors-2.drawio.svg)
