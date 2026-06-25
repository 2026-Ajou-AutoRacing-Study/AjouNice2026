---
source_type: rendered_autoware_docs
site: autoware_universe
title: Index
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_remaining_distance_time_calculator/
rendered_html: rendered/autoware_universe_html/index-167.html
generated: 2026-06-24T14:59:32.283184+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Index

## Remaining Distance and Time Calculator

### Role

This package aims to provide mission remaining distance and remaining time calculations.

### Activation and Timing

- The calculations are activated once we have a route planned for a mission in Autoware.
- The calculations are triggered timely based on the `update_rate` parameter.
- The calculations are skipped if the scenario is PARKING, and the remaining time and distance values are set to 0.0.

### Module Parameters

| Name | Type | Default Value | Explanation |
| --- | --- | --- | --- |
| `update_rate` | double | 10.0 | Timer callback period. [Hz] |

### Inner-workings

#### Remaining Distance Calculation

- The remaining distance calculation is based on getting the remaining shortest path between the current vehicle pose and goal pose using `lanelet2` routing APIs.
- The remaining distance is calculated by summing the 2D length of remaining shortest path, with exception to current lanelet and goal lanelet.
  - For the current lanelet, the distance is calculated from the current vehicle position to the end of that lanelet.
  - For the goal lanelet, the distance is calculated from the start of the lanelet to the goal pose in this lanelet.
- When there is only one lanelet remaining, the distance is calculated by getting the 2D distance between the current vehicle pose and goal pose.
- Checks are added to handle cases when current lanelet, goal lanelet, or routing graph are not valid to prevent node process die.
  - In such cases when, last valid remaining distance and time are maintained.

#### Remaining Time Calculation

- The remaining time currently depends on a simple equation of motion by getting the maximum velocity limit.
- The remaining distance is calculated by dividing the remaining distance by the maximum velocity limit.
- A check is added to the remaining time calculation to make sure that maximum velocity limit is greater than zero. This prevents division by zero or getting negative time value.

### Future Work

- Find a more efficient way for remaining distance calculation instead of regularly searching the graph for finding the remaining shortest path.
- Engage more sophisticated motion models for more accurate remaining time calculations.
