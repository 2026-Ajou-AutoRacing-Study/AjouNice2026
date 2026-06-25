---
source_type: rendered_autoware_docs
site: autoware_universe
title: Bidirectional Traffic
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/
rendered_html: rendered/autoware_universe_html/index-187.html
generated: 2026-06-24T14:59:33.512718+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Bidirectional Traffic

This module enables trajectory planning for vehicles on "single-lane bidirectional traffic" roads in autonomous driving systems. Here, "single-lane bidirectional traffic" refers to roads without a centerline, such as those commonly found in residential areas, as shown in the image below．

![BidirectionalTrafficShiojiri](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/BidirectionalTrafficShiojiri.png)

In the following descriptions, the term bidirectional traffic refers specifically to such single-lane bidirectional roads as shown in the image above.

## Purpose/Role

The main roles of this module are as follows:

- Generating a driving trajectory along the left (or right) side of the road center

  This module provides a function to generate a trajectory along the left (or right) side of the road center on bidirectional traffic lanes. In some countries, traffic laws require vehicles to drive on the left or right side of the road center on bidirectional roads. For example, in Japan, [Article 18, Paragraph 1 of the Road Traffic Act](https://laws.e-gov.go.jp/law/335AC0000000105#Mp-Ch_3-Se_1-At_18) requires that vehicles must keep to the left side of the road center. Because of these legal requirements, this module provides the function to generate a trajectory that follows the left (or right) side of the road center on bidirectional roads.

- Giving way to oncoming vehicles on bidirectional traffic roads

  This module provides the function for the ego vehicle to stop on the left side of the road and give way to oncoming vehicles in all situations where vehicles must pass each other on bidirectional roads. Bidirectional traffic roads are often narrow, and it is dangerous for both the ego vehicle and the oncoming vehicle to pass while moving simultaneously. To avoid such dangerous situations, this module enables the ego vehicle to stop on the left side of the road until the oncoming vehicle has completely passed.

## The representation of bidirectional traffic on a Lanelet map

Bidirectional lanelets are represented on a Lanelet map as two lanes with opposite directions that share the same LineString as their boundary. There is no need to assign any special tags to the lanelets.

![BidirectionalTrafficMap](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/BidirectionalTrafficMap.png)

## Limitations

This module does not support the following situations or functionalities.

- Handling situations where passing is not possible

  Depending on factors such as lane width, the size of the ego vehicle and the oncoming vehicle, and the surrounding environment, simple behaviors like those shown in the video above may not be sufficient for passing. In such cases, special behaviours such as reversing or clearing an intersection may be required. However, this module does not provide such functionality.

- When the oncoming vehicle is giving way or moving slowly

  In cases where the oncoming vehicle is trying to give way or is traveling at a low speed, it may be more appropriate for the ego vehicle to continue driving rather than stopping to give way. However, this module does not make any decision about whether to give way to the oncoming vehicle -- **it always pulls over to the left and stops.**

- Does not support the functionality of pulling into the shoulder to stop and give way

  Depending on the road width, the ego vehicle may need to enter the shoulder in order to give way to an oncoming vehicle. However, this module does not support such functionality.

## Inner workings/Algorithms

This module does not support the following situations or functionalities.

### Keep Left

This module detects bidirectional lanelets from the lanelet map and performs a keep left maneuver. When keeping left, the shift starts from the beginning of the lanelet immediately before the bidirectional lanelet, and ends at the end of the bidirectional lanelet.

![KeepLeft](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/KeepLeft.drawio.svg)

### Give Way

#### State transition of GiveWay

To perform the give way maneuver, the GiveWay module has five internal states.

- **NoNeedToGiveWay**

  No oncoming vehicles present and driving normally or oncoming vehicles are far away.

  ![NoNeedToGiveWay](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/NoNeedToGiveWay.drawio.svg)

- **ApproachingToShift**

  If an oncoming vehicle is detected, it generates a path to shift left and pull over, approaching the shift start point.

  ![ApproachingToShift](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/ApproachingToShift.drawio.svg)

- **ShiftingRoadside**

  ![ShiftingRoadside](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/ShiftingRoadside.drawio.svg)

- **WaitingForOncomingCarsToPass**

  The ego vehicle stops and waits until all oncoming vehicles have passed.

  ![WaitingForOncomingCarsToPass](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/WaitingForOncomingCarsToPass.drawio.svg)

- **BackToNormalLane**

  After the oncoming vehicle has passed, the ego vehicle performs a shift to return to its original driving lane.

  ![BackToNormalLane](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/BackToNormalLane.drawio.svg)

The state transition diagram of the GiveWay module is shown below.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/image-01-37dfb2ea8a.svg)

#### How to decide shift length to pull over?

When an oncoming vehicle is detected, it is necessary to determine how much longitudinal distance to consume for the shift.

![ShiftDistance](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/assets/ShiftDistance.drawio.svg)

The figure above illustrates the distances related to lane shifting. The shift prepare distance is the distance required to prepare for shifting before reaching the shift start point, and it is calculated by multiplying the parameter time\_to\_prepare\_pull\_over by the ego\_velocity.

These distances are calculated when the state is NoNeedToGiveWay. Once the ego vehicle enters the shift prepare distance, the state transitions to ApproachingToShift. When it enters the shift distance, the state transitions to ShiftingRoadside.

While in the NoNeedToGiveWay state, the calculation of the shift\_distance and the corresponding state transitions are handled as follows.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/image-02-aa891b6b60.svg)

We now provide a detailed explanation of the following four calculations:

##### 1. Calculate `lateral_shift_distance`

`lateral_shift_distance` is the lateral distance that the ego vehicle needs to shift. This value is calculated from `min_distance_from_roadside` and `shift_distance_to_pull_over_from_center_line`, based on `road_width` and `vehicle_width`, as follows:

\[
\text{lateral\_shift\_distance} = \text{max}(\text{road\_width} / 2 - \text{vehicle\_width} / 2 - \text{min\_distance\_from\_roadside}, \text{shift\_distance\_to\_pull\_over\_from\_center\_line})
\]

##### 2. Calculate `allowed_longitudinal_distance`

`allowed_longitudinal_distance` is the minimum longitudinal distance required for the ego vehicle to perform the shift. Shifting in a shorter distance would require a sharp steering maneuver, which is unsafe. This distance is calculated such that the lateral jerk derived from `lateral_shift_distance` and `ego_velocity` does not exceed `max_lateral_jerk`:

\[
\text{allowed\_longitudinal\_distance} = \text{calc\_longitudinal\_dist\_from\_jerk}(\text{lateral\_shift\_distance}, \text{max\_lateral\_jerk}, \text{ego\_velocity})
\]

##### 3. Calculate `desired_pull_over_point`

If the shift starts too early, the ego vehicle may have to wait a long time for the oncoming vehicle to pass. If it starts too late, there is a risk of collision. To avoid both, the ideal point to begin the pull-over is calculated based on the ego speed and the oncoming vehicle's speed, assuming the ego vehicle will stop and wait for `wait_time_for_oncoming_car` seconds:

\[
\text{desired\_pull\_over\_point} = \frac{(\text{distance\_to\_oncoming\_car} - \text{oncoming\_car\_speed} \cdot \text{wait\_time\_for\_oncoming\_car}) \cdot \text{ego\_speed}}{\text{ego\_speed} + \text{oncoming\_car\_speed}}
\]

##### 4. Calculate `max_longitudinal_distance`

An overly long shift distance may appear unnatural. To prevent this, the maximum longitudinal distance is calculated based on the minimum lateral jerk:

\[
\text{max\_longitudinal\_distance} = \text{calc\_longitudinal\_dist\_from\_jerk}(\text{lateral\_shift\_distance}, \text{min\_lateral\_jerk}, \text{ego\_velocity})
\]

## Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| time\_to\_prepare\_pull\_over | float | Time to prepare for pull over. When an oncoming vehicle is detected, ego\_speed x time\_to\_prepare\_pull\_over [m] starts shifting from ahead. Unit is **[s]**. | 1 | ≥0.0 |
| min\_distance\_from\_roadside | float | Minimum distance from left roadside to pull over. Unit is **[m]**. | 0.2 | ≥0.0 |
| keep\_left\_distance\_from\_center\_line | float | Distance to keep left from center line. Unit is **[m]**. | 0.5 | ≥0.0 |
| shift\_distance\_to\_pull\_over\_from\_center\_line | float | Distance to shift to the left from center line when pulling over. Unit is **[m]**. | 1.2 | ≥0.0 |
| wait\_time\_for\_oncoming\_car | float | Wait time for oncoming car from ego vehicle stops to ego vehicle starts. Unit is **[s]**. | 15 | ≥0.0 |
| max\_lateral\_jerk | float | Maximum lateral jerk when pulling over. Unit is **[m/s^3]**. | 3 | ≥0.0 |
| min\_lateral\_jerk | float | Minimum lateral jerk when pulling over. Unit is **[m/s^3]**. | 2 | ≥0.0 |
