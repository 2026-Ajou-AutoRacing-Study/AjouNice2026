---
source_type: rendered_autoware_docs
site: autoware_universe
title: Index
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module/
rendered_html: rendered/autoware_universe_html/index-208.html
generated: 2026-06-24T14:59:35.129145+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Index

## No Drivable Lane

### Role

This module plans the velocity of the related part of the path in case there is a no drivable lane referring to it.

A no drivable lane is a lanelet or more that are out of operation design domain (ODD), i.e., the vehicle **must not** drive autonomously in this lanelet.  
A lanelet can be no drivable (out of ODD) due to many reasons, either technical limitations of the SW and/or HW, business requirements, safety considerations, .... etc, or even a combination of those.

Some examples of No Drivable Lanes

- Closed road intentionally, due to construction work for example
- Underpass road that goes under a railway, for safety reasons
- Road with slope/inclination that the vehicle is not be able to drive autonomously due to technical limitations. And lots of other examples.

![no-drivable-lane-design.svg](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module/docs/no_drivable_lane_design.svg)

A lanelet becomes invalid by adding a new tag under the relevant lanelet in the map file `<tag k="no_drivable_lane" v="yes"/>`.

The target of this module is to stop the vehicle before entering the no drivable lane (with configurable stop margin) or keep the vehicle stationary if autonomous mode started inside a no drivable lane. Then ask the human driver to take the responsibility of the driving task (Takeover Request / Request to Intervene)

### Activation Timing

This function is activated when the lane id of the target path has an no drivable lane label (i.e. the `no_drivable_lane` attribute is `yes`).

### Module Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `stop_margin` | double | [m] margin for ego vehicle to stop before speed\_bump |
| `print_debug_info` | bool | whether debug info will be printed or not |

### Inner-workings / Algorithms

- Get no\_drivable\_lane attribute on the path from lanelet2 map
- The no drivable lane state machine starts in `INIT` state
- Get the intersection points between path and no drivable lane polygon
- Assign the state to `APPROACHING` toward a no drivable lane if:
  - the distance from front of the ego vehicle till the first intersection point between the ego path and the no drivable lane polygon is more than the `stop_margin`
- Assign the state to `INSIDE_NO_DRIVABLE_LANE` if:
  - the first point of the ego path is inside the no drivable lane polygon, or
  - the distance from front of the ego vehicle till the first intersection point between the ego path and the no drivable lane polygon is less than the `stop_margin`
- Assign the state to `STOPPED` when the vehicle is completely stopped

![no_drivable_lane_scenarios.svg](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module/docs/no_drivable_lane_scenarios.svg)

### Future Work

- As [Request to Intervene API](https://github.com/autowarefoundation/autoware/issues/3487) is not implemented yet, this will be handled to notify the driver to takeover the driving task responsibility after the vehicle stops due to `no_drivable_lane`
- Handle the case when the vehicle stops before a no drivable lane but part of its footprint intersects with the no drivable lane polygon.
