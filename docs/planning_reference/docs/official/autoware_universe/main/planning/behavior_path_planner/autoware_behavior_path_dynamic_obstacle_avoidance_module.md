---
source_type: rendered_autoware_docs
site: autoware_universe
title: Avoidance module for dynamic objects
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/
rendered_html: rendered/autoware_universe_html/index-189.html
generated: 2026-06-24T14:59:33.695443+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Avoidance module for dynamic objects

This module is under development.

## Purpose / Role

This module provides avoidance functions for vehicles, pedestrians, and obstacles in the vicinity of the ego's path in combination with the [autoware\_path\_optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/).
Each module performs the following roles.
Dynamic Avoidance module cuts off the drivable area according to the position and velocity of the target to be avoided.
Obstacle Avoidance module modifies the path to be followed so that it fits within the received drivable area.

Static obstacle's avoidance functions are also provided by the [Static Avoidance module](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/), but these modules have different roles.
The Static Obstacle Avoidance module performs avoidance through the outside of own lanes but cannot avoid the moving objects.
On the other hand, this module can avoid moving objects.
For this reason, the word "dynamic" is used in the module's name.
The table below lists the avoidance modules that can handle each situation.

|  | avoid within the own lane | avoid through the outside of own lanes |
| --- | --- | --- |
| avoid not-moving objects | Avoidance Module   Dynamic Avoidance Module + Obstacle Avoidance Module | Avoidance Module |
| avoid moving objects | Dynamic Avoidance Module + Obstacle Avoidance Module | No Module (Under Development) |

## Policy of algorithms

Here, we describe the policy of inner algorithms.
The inner algorithms can be separated into two parts: The first decides whether to avoid the obstacles and the second cuts off the drivable area against the corresponding obstacle.

### Select obstacles to avoid

To decide whether to avoid an object, both the predicted path and the state (pose and twist) of each object are used.
The type of objects the user wants this module to avoid is also required.
Using this information, the module decides to *avoid* objects that *obstruct the ego's passage* and *can be avoided*.

The definition of *obstruct the ego's passage* is implemented as the object that collides within seconds.
The other, *can be avoided* denotes whether it can be avoided without risk to the passengers or the other vehicles.
For this purpose, the module judges whether the obstacle can be avoided with satisfying the constraints of lateral acceleration and lateral jerk.
For example, the module decides not to avoid an object that is too close or fast in the lateral direction.

### Cuts off the drivable area against the selected vehicles

For the selected obstacles to be avoided, the module cuts off the drivable area.
As inputs to decide the shapes of cut-off polygons, poses of the obstacles are mainly used, assuming they move in parallel to the ego's path, instead of its predicted path.
This design arises from that the predicted path of objects is not accurate enough to use the path modifications (at least currently).
Furthermore, the output drivable area shape is designed as a rectangular cutout along the ego's path to make the computation scalar rather than planar.

#### Determination of lateral dimension

The lateral dimensions of the polygon are calculated as follows.
The polygon's width to extract from the drivable area is the obstacle width and `drivable_area_generation.lat_offset_from_obstacle`.
We can limit the lateral shift length by `drivable_area_generation.max_lat_offset_to_avoid`.

![drivable_area_extraction_width](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/drivable_area_extraction_width.drawio.svg)

#### Determination of longitudinal dimension

Then, extracting the same directional and opposite directional obstacles from the drivable area will work as follows considering TTC (time to collision).

Regarding the same directional obstacles, obstacles whose TTC is negative will be ignored (e.g., The obstacle is in front of the ego, and the obstacle's velocity is larger than the ego's velocity.).

Same directional obstacles (Parameter names may differ from implementation)
![same_directional_object](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/same_directional_object.svg)

Opposite directional obstacles (Parameter names may differ from implementation)
![opposite_directional_object](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/opposite_directional_object.svg)

### Cuts off the drivable area against the selected pedestrians

Then, we describe the logic to generate the drivable areas against pedestrians to be avoided.
Objects of this type are considered to have priority right of way over the ego's vehicle while ensuring a minimum safety of the ego's vehicle.
In other words, the module assigns a drivable area to an obstacle with a specific margin based on the predicted paths with specific confidences for a specific time interval, as shown in the following figure.

![](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/2024-04-18_15-13-01.png)


Restriction areas are generated from each pedestrian's predicted paths

Apart from polygons for objects, the module also generates another polygon to ensure the ego's safety, i.e., to avoid abrupt steering or significant changes from the path.
This is similar to avoidance against the vehicles and takes precedence over keeping a safe distance from the object to be avoided.
As a result, as shown in the figure below, the polygons around the objects reduced by the secured polygon of the ego are subtracted from the ego's drivable area.

![](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/2024-04-18_15-32-03.png)


Ego's minimum requirements are prioritized against object margin

## Example

![](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/image-20230807-151945.png)


Avoidance for the bus departure


![](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/image-20230807-152835.png)


Avoidance on curve


![](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/image-20230808-095936.png)


Avoidance against the opposite direction vehicle


![](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/image/image-20230808-152853.png)


Avoidance for multiple vehicle

## Future works

Currently, the path shifting length is limited to 0.5 meters or less by `drivable_area_generation.max_lat_offset_to_avoid`.
This is caused by the lack of functionality to work with other modules and the structure of the planning component.
Due to this issue, this module can only handle situations where a small avoidance width is sufficient.
This issue is the most significant for this module.
In addition, the ability of this module to extend the drivable area as needed is also required.

## Parameters

Under development

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| target\_object.car | [-] | bool | The flag whether to avoid cars or not | true |
| target\_object.truck | [-] | bool | The flag whether to avoid trucks or not | true |
| ... | [-] | bool | ... | ... |
| target\_object.min\_obstacle\_vel | [m/s] | double | Minimum obstacle velocity to avoid | 1.0 |
| drivable\_area\_generation.lat\_offset\_from\_obstacle | [m] | double | Lateral offset to avoid from obstacles | 0.8 |
| drivable\_area\_generation.max\_lat\_offset\_to\_avoid | [m] | double | Maximum lateral offset to avoid | 0.5 |
| drivable\_area\_generation.overtaking\_object.max\_time\_to\_collision | [s] | double | Maximum value when calculating time to collision | 3.0 |
| drivable\_area\_generation.overtaking\_object.start\_duration\_to\_avoid | [s] | double | Duration to consider avoidance before passing by obstacles | 4.0 |
| drivable\_area\_generation.overtaking\_object.end\_duration\_to\_avoid | [s] | double | Duration to consider avoidance after passing by obstacles | 5.0 |
| drivable\_area\_generation.overtaking\_object.duration\_to\_hold\_avoidance | [s] | double | Duration to hold avoidance after passing by obstacles | 3.0 |
| drivable\_area\_generation.oncoming\_object.max\_time\_to\_collision | [s] | double | Maximum value when calculating time to collision | 3.0 |
| drivable\_area\_generation.oncoming\_object.start\_duration\_to\_avoid | [s] | double | Duration to consider avoidance before passing by obstacles | 9.0 |
| drivable\_area\_generation.oncoming\_object.end\_duration\_to\_avoid | [s] | double | Duration to consider avoidance after passing by obstacles | 0.0 |
