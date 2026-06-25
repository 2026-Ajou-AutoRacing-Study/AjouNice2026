---
source_type: rendered_autoware_docs
site: autoware_universe
title: Manual Lane Change Handler
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_manual_lane_change_handler/
rendered_html: rendered/autoware_universe_html/index-160.html
generated: 2026-06-24T14:59:31.962304+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Manual Lane Change Handler

## Purpose

`Manual Lane Change Handler` helps shift the preferred-primitives of a route to the left or right. A manual lane-change handler rviz plugin is also available for use for the user.

### Example for using the Manual Lane Change Handler

Service call to shift the preferred-lanes to the left

```
ros2 service call /planning/mission_planning/manual_lane_change_handler/set_preferred_lane tier4_external_api_msgs/srv/SetPreferredLane lane_change_direction:\ 0\
```

### Services

| Name | Type | Description |
| --- | --- | --- |
| `/planning/manual_lane_change_handler/manual_lane_change_handler/set_preferred_lane` | tier4\_external\_api\_msgs/srv/SetPreferredLane | preferred-lane request |

### Subscriptions

| Name | Type | Description |
| --- | --- | --- |
| `input/odometry` | nav\_msgs/msg/Odometry | vehicle odometry |
| `/planning/mission_planning/route` | autoware\_planning\_msgs/msg/LaneletRoute | current lanelet route |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/is_reroute_available` | tier4\_planning\_msgs/msg/RerouteAvailability | Availability to reroute generally published by the behavior\_path\_planner. The `set_preferred_lane` service will be denied when this topic is `false`. |

### Publications

| Name | Type | Description |
| --- | --- | --- |
| `~/debug/processing_time_ms` | autoware\_internal\_debug\_msgs/msg/Float64Stamped | processing time for lane change for debug |
| `~/debug/route_marker` | visualization\_msgs/msg/MarkerArray | route marker for debug |
| `~/debug/goal_footprint` | visualization\_msgs/msg/MarkerArray | goal footprint for debug |

## Service client

| Service Name | Description |
| --- | --- |
| ~/set\_preferred\_lane | The service to set the preferred lane |

## Registered Servers

| Service Name | Description |
| --- | --- |
| /planning/mission\_planning/mission\_planner/set\_preferred\_primitive | The service to set the preferred primitive |

### Setting Preferred Lane

This service allows for shifting the current preferred lane to the left or right, or reverting to the default preferred lane received from the route planner.
As the vehicle moves, the shifting is done relative to the current preferred lane.

Based on certain criteria, shifting may be rejected, as in the following cases:

1. Left or Right shift is not available due to no lane being present to shift to
2. The next segment is a turn or the very last lane - this is to ensure that we can navigate the enter path and end up at the goal
3. The reroute availability provided by the behavior\_path\_planner is `false`

![uml diagram](../../../../assets/autoware_universe/main/planning/autoware_manual_lane_change_handler/image-01-c32deb9957.svg)
