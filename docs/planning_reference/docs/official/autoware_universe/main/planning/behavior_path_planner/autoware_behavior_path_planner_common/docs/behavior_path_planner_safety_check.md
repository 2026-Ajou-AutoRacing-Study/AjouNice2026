---
source_type: rendered_autoware_docs
site: autoware_universe
title: Safety Check Utils
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_safety_check/
rendered_html: rendered/autoware_universe_html/index-198.html
generated: 2026-06-24T14:59:34.346204+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Safety Check Utils

The safety check function determines if a specific path will result in a collision with a target object.

## Purpose / Role

In the Behavior Path Planner, modules such as Lane Change require collision checks to ensure safe navigation. The safety check utility functions assist the module in conducting safety checks with other road participants. This approach ensures collision avoidance by considering the ego vehicle’s braking distance, other vehicles’ braking distances, and minimum safety distances as part of the safety design.

### Assumptions

The safety check module operates based on the following assumptions:

1. The module must receive the position, velocity, and shape for both the ego vehicle and the target objects.
2. For every point in the predicted path of the ego and target objects, the yaw angle (direction) must point toward the next point in that path.
3. The module uses the minimum safe braking distance to determine if a collision risk exists with other objects.

### Limitations

Currently, the yaw angle of points in a target object's predicted path may not always point exactly to the next point. Therefore, the safety check function might return incorrect results in some specific edge cases.

### Inner working / Algorithm

The flow of the safety check algorithm is described in the following explanations.

![safety_check_flow](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/images/path_safety_checker/safety_check_flow.drawio.svg)

Here we explain each step of the algorithm flow.

#### Step 1: Obtain the target object position (pose) at a given time

The module first determines the position of the target object at a specific time. This is calculated by interpolating the predicted path of the object.

#### Step 2: Check for initial overlap

Using the interpolated position from Step 1, the module checks if the target object and ego vehicle occupy the same space at that time. If they overlap, the path is marked unsafe.

#### Step 3: Identify the leading vehicle

If no initial overlap is found, the module determines which vehicle is in front of the other. The module compares the arc length (distance along the path) of the front point of each object.

Example: When the target object (red rectangle) has a greater arc length than the ego vehicle (black rectangle), it is considered the leading vehicle.

![front_object](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/images/path_safety_checker/front_object.drawio.svg)

#### Step 4: Calculate the Minimum Safe Braking Distance

Once the leading vehicle is identified, the module calculates the safe braking distance. Using the reaction time (\(t\_{reaction}\)) and the safety time margin (\(t\_{margin}\)), the distance (\(d\_{braking}\)) is defined as:

\[
d\_{braking} = v\_{rear} (t\_{reaction} + t\_{margin}) + \frac{v\_{rear}^2}{2|a\_{rear, decel}|} - \frac{v\_{front}^2}{2|a\_{front, decel|}}
\]

Where:

- \(v\_{front}\) and \(v\_{rear}\) are the velocities of the front and rear vehicles.
- \(a\_{front, decel}\) and \(a\_{rear, decel}\) are the maximum decelerations of the front and rear vehicles.

Note

Note: This safety check is typically used for objects traveling in the same direction. If the difference in yaw angle (direction) between the ego vehicle and the object exceeds a user-defined threshold, the module skips the check for that pair of positions.

#### Step 5: Create extended ego and target object polygons

The module creates geometric shapes (polygons) to represent the safety zones. The module expands the rear object polygon based on the following:

- Longitudinally: It extends the polygon forward by the calculated safe braking distance (\(d\_{braking}\)).
- Laterally: It extends the polygon sideways by a lateral margin.

![extended_polygons](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/images/path_safety_checker/extended_polygons.drawio.svg)

#### Step 6: Final overlap check

The module checks if the extended rear polygon overlaps with the front object polygon. If an overlap is detected, the situation is determined to be unsafe.
