---
source_type: rendered_autoware_docs
site: autoware_universe
title: Start Planner design
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/
rendered_html: rendered/autoware_universe_html/index-202.html
generated: 2026-06-24T14:59:34.629037+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Start Planner design

## Purpose / Role

This module generates and plans a path for safely merging from the shoulder lane or side of road lane into the center of the road lane.

Specifically, it includes the following features:

- Plan the path to automatically start from the shoulder lane or side of road lane to center of road lane.
- When parked vehicles are present on the shoulder lane, the module generates a path that allows for starting with a gap of a specified margin.
- If a collision with other traffic participants is detected while traveling on the generated path, it will stop as much as possible.

![start_planner_module](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/start_planner_example.png)

## Use Cases

Give an typical example of how path generation is executed. Showing example of path generation starts from shoulder lane, but also from side of road lane can be generated.

![shift_pull_out_from_road_lane](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/shift_pull_out_path_from_road_lane.drawio.svg)

### **Use Case 1: Shift pull out**

In the shoulder lane, when there are no parked vehicles ahead and the shoulder lane is sufficiently long, a forward shift pull out path (a clothoid curve with consistent jerk) is generated.

![shift_pull_out](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/shift_pull_out_path.drawio.svg)

### **Use Case 2: Geometric pull out**

In the shoulder lane, when there are objects in front and the lane is not sufficiently long behind, a geometric pull out path is generated.

![geometric_pull_out](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/geometric_pull_out_path.drawio.svg)

### **Use Case 3: Backward and shift pull out**

In the shoulder lane, when there are parked vehicles ahead and the lane is sufficiently long behind, a path that involves reversing before generating a forward shift pull out path is created.

![shift_pull_out_with_back](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/shift_pull_out_path_with_back.drawio.svg)

### **Use Case 4: Backward and geometric pull out**

In the shoulder lane, when there is an object ahead and not enough space to reverse sufficiently, a path that involves reversing before making an geometric pull out is generated.

![geometric_pull_out_with_back](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/geometric_pull_out_path_with_back.drawio.svg)

### **Use Case 5: Clothoid pull out**

This method creates smooth paths using clothoid curves that provide continuous curvature transitions.

### **Use Case 6: Freespace pull out**

If the map is annotated with the information that a free space path can be generated in situations where both shift and geometric pull out paths are impossible to create, a path based on the free space algorithm will be generated.

![freespace_path](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/freespace_pull_out.png)

**As a note, the patterns for generating these paths are based on default parameters, but as will be explained in the following sections, it is possible to control aspects such as making paths that involve reversing more likely to be generated, or making geometric paths more likely to be generated, by changing the path generation policy or adjusting the margin around static objects.**

## Limitations

Here are some notable limitations:

- If parked vehicles exist in front of, behind, or on both sides of the shoulder, and it's not possible to maintain a specified distance from them, a stopping path will be generated, leading to a potential deadlock.
- In the default settings of the behavior\_path\_planner, an avoidance module operates in a later stage and attempts to avoid objects. However, it is not guaranteed that the start\_planner module will output a path that can successfully navigate around obstacles. This means that if an unavoidable path is generated, it could result in a deadlock.
- The performance of safety check relies on the accuracy of the predicted paths generated by the map\_based\_prediction node. It's important to note that, currently, predicted paths that consider acceleration are not generated, and paths for low-speed objects may not be accurately produced, which requires caution.
- Given the current specifications of the rule-based algorithms, there is a trade-off between the safety of a path and its naturalness to humans. While it's possible to adjust parameters to manage this trade-off, improvements are necessary to better reconcile these aspects.

## Start/End Conditions

### **Start Conditions**

The `StartPlannerModule` is designed to initiate its execution based on specific criteria evaluated by the `isExecutionRequested` function. The module will **not** start under the following conditions:

1. **Start pose on the middle of the road**: The module will not initiate if the start pose of the vehicle is determined to be in the middle of the road. This ensures the planner starts from a roadside position.
2. **Vehicle is far from start position**: If the vehicle is far from the start position, the module will not execute. This prevents redundant execution when the new goal is given.
3. **Vehicle reached goal**: The module will not start if the vehicle has already reached its goal position, avoiding unnecessary execution when the destination is attained.
4. **Vehicle in motion**: If the vehicle is still moving, the module will defer starting. This ensures that planning occurs from a stable, stationary state for safety.
5. **Goal behind in same route segment**: The module will not initiate if the goal position is behind the ego vehicle within the same route segment. This condition is checked to avoid complications with planning routes that require the vehicle to move backward on its current path, which is currently not supported.

### **End Conditions**

The `StartPlannerModule` terminates when specific conditions are met, depending on the type of planner being used. The `canTransitSuccessState` function determines whether the module should transition to the success state based on the following criteria:

#### When the Freespace Planner is active

- If the end point of the freespace path is reached, the module transitions to the success state.

#### When any other type of planner is active

The transition to the success state is determined by the following conditions:

- If a reverse path is being generated or the search for a pull out path fails:
  - The module does not transition to the success state.
- If the end point of the pull out path's shift section is reached:
  - The module transitions to the success state.

The flowchart below illustrates the decision-making process in the `canTransitSuccessState` function:

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/image-01-b4fa91409c.svg)

## Concept of safety assurance

The approach to collision safety is divided into two main components: generating paths that consider static information, and detecting collisions with dynamic obstacles to ensure the safety of the generated paths.

### 1. Generating path with static information

- **Path deviation checks**: This ensures that the path remains within the designated lanelets. By default, this feature is active, but it can be deactivated if necessary.

- **Static obstacle clearance from the path**: This involves verifying that a sufficient margin around static obstacles is maintained. The process includes creating a vehicle-sized footprint from the current position to the pull-out endpoint, which can be adjusted via parameters. The distance to static obstacle polygons is then calculated. If this distance is below a specified threshold, the path is deemed unsafe. Threshold levels (e.g., [2.0, 1.0, 0.5, 0.1]) can be configured, and the system searches for paths that meet the highest possible threshold based on a set search priority explained in following section, ensuring the selection of the safe path based on the policy. If no path meets the minimum threshold, it's determined that no safe path is available.

- **Clearance from stationary objects**: Maintaining an adequate distance from stationary objects positioned in front of and behind the vehicle is imperative for safety. Despite the path and stationary objects having a confirmed margin, the path is deemed unsafe if the distance from the shift start position to a front stationary object falls below `collision_check_margin_from_front_object` meters, or if the distance to a rear stationary object is shorter than `back_objects_collision_check_margin` meters.

  - Why is a margin from the front object necessary?
    Consider a scenario in a "geometric pull out path" where the clearance from the path to a static obstacle is minimal, and there is a stopped vehicle ahead. In this case, although the path may meet safety standards and thus be generated, a concurrently operating avoidance module might deem it impossible to avoid the obstacle, potentially leading to vehicle deadlock. To ensure there is enough distance for avoidance maneuvers, the distance to the front obstacle is assessed. Increasing this parameter can prevent immobilization within the avoidance module but may also lead to the frequent generation of backward paths or geometric pull out path, resulting in paths that may seem unnatural to humans.
  - Why is a margin from the rear object necessary?
    For objects ahead, another behavior module can intervene, allowing the path to overwrite itself through an avoidance plan, even if the clearance from the path to a static obstacle is minimal, thus maintaining a safe distance from static obstacles. However, for objects behind the vehicle, it is impossible for other behavior modules other than the start\_planner to alter the path to secure a margin, potentially leading to a deadlock by an action module like "obstacle\_stop\_module" and subsequent immobilization. Therefore, a margin is set for stationary objects at the rear.

Here's the expression of the steps start pose searching steps, considering the `collision_check_margins` is set at [2.0, 1.0, 0.5, 0.1] as example. The process is as follows:

1. **Generating start pose candidates**

   - Set the current position of the vehicle as the base point.
   - Determine the area of consideration behind the vehicle up to the `max_back_distance`.
   - Generate candidate points for the start pose in the backward direction at intervals defined by `backward_search_resolution`.
   - Include the current position as one of the start pose candidates.

   ![start pose candidate](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/start_pose_candidate.drawio.svg)
2. **Starting search at maximum margin**

   - Begin the search with the largest threshold (e.g., 2.0 meters).
   - Evaluate each start pose candidate to see if it maintains a margin of more than 2.0 meters.
   - Simultaneously, verify that the path generated from that start pose meets other necessary criteria (e.g., path deviation check).
   - Following the search priority described later, evaluate each in turn and adopt the start pose if it meets the conditions.
3. **Repeating search according to threshold levels**

   - If no start pose meeting the conditions is found, lower the threshold to the next level (e.g., 1.0 meter) and repeat the search.
4. **Continuing the search**

   - Continue the search until a start pose that meets the conditions is found, or the threshold level reaches the minimum value (e.g., 0.1 meter).
   - The aim of this process is to find a start pose that not only secures as large a margin as possible but also satisfies the conditions required for the path.
5. **Generating a stop path**

   - If no start pose satisfies the conditions at any threshold level, generate a stop path to ensure safety.

#### **search priority**

If a safe path with sufficient clearance for static obstacles cannot be generated forward, a backward search from the vehicle's current position is conducted to locate a suitable start point for a pull out path generation.

During this backward search, different policies can be applied based on `search_policy` parameter:

Selecting `planner_priority` focuses on creating efficient paths by trying all candidates for each planner type first.
Opting for `distance_priority` aims to find a location with the least possible backward movement by trying all planners for each candidate first.

![priority_order](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/priority_order.drawio.svg)

`PriorityOrder` is defined as a vector of pairs, where each element consists of a `size_t` index representing a start pose candidate index, and the planner type. The PriorityOrder vector is processed sequentially from the beginning, meaning that the pairs listed at the top of the vector are given priority in the selection process for pull out path generation.

##### `planner_priority`

When `search_policy` is set to `planner_priority`, the `PriorityOrder` array is populated in such a way that all start pose candidates are tried for each planner type before moving on to the next planner type. This prioritization is reflected in the order of the array, with all candidates for the first planner being listed before any candidates for the second planner.

| Index | Planner Type |
| --- | --- |
| 0 | shift\_pull\_out |
| 1 | shift\_pull\_out |
| ... | ... |
| N | shift\_pull\_out |
| 0 | geometric\_pull\_out |
| 1 | geometric\_pull\_out |
| ... | ... |
| N | geometric\_pull\_out |

This approach prioritizes trying all candidates with each planner type before proceeding to the next planner type, which may be efficient in situations where certain planner types are more likely to be appropriate.

##### `distance_priority`

For `search_policy` set to `distance_priority`, the array alternates between planner types for each start pose candidate, which can minimize the distance the vehicle needs to move backward if the earlier candidates are successful.

| Index | Planner Type |
| --- | --- |
| 0 | shift\_pull\_out |
| 0 | geometric\_pull\_out |
| 1 | shift\_pull\_out |
| 1 | geometric\_pull\_out |
| ... | ... |
| N | shift\_pull\_out |
| N | geometric\_pull\_out |

This ordering is beneficial when the priority is to minimize the backward distance traveled, giving an equal chance for each planner to succeed at the closest possible starting position.

### 2. Collision detection with dynamic obstacles

- Using Safe Braking Distances for Collision Assessment: The module uses a safe braking distance model to determine if the ego vehicle is maintaining enough space from other dynamic objects. See [safety check feature explanation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_safety_check/)

- **Collision check performed range**: Safety checks for collisions with dynamic objects are conducted within the defined boundaries between the start and end points of each maneuver, ensuring the ego vehicle does not impede or hinder the progress of dynamic objects that come from behind it.

- **Collision response policy**: Should a collision with dynamic objects be detected along the generated path, deactivate module decision is registered if collision detection occurs before departure. If the vehicle has already commenced movement, an attempt to stop will be made, provided it's feasible within the braking constraints and that the rear vehicle can pass through the gap between the ego vehicle and the lane border.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/image-02-fd9c730ebd.svg)

#### **example of safety check performed range for shift pull out**

Give an example of safety verification range for shift pull out. The safety check is performed from the start of the shift to the end of the shift. And if the vehicle footprint does not leave enough space for a rear vehicle to drive through, the safety check against dynamic objects is disabled.

![safety check performed range for shift pull out](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/collision_check_range_shift_pull_out.drawio.svg)

**As a note, no safety check is performed during backward movements. Safety verification commences at the point where the backward motion ceases.**

## RTC interface

The system operates distinctly under manual and auto mode, especially concerning the before the departure and interaction with dynamic obstacles. Below are the specific behaviors for each mode:

### When approval is required?

#### Forward driving

- **Start approval required**: Even if a path is generated, approval is required to initiate movement. If a dynamic object poses a risk, such as an approaching vehicle from behind, candidate paths may be displayed, but approval is necessary for departure.

#### Backward driving + forward driving

- **Multiple approvals required**: When the planned path includes a backward driving, multiple approvals are needed before starting the reverse and again before restarting forward movement. Before initiating forward movement, the system conducts safety checks against dynamic obstacles. If a detection is detected, approval for departure is necessary.

This differentiation ensures that the vehicle operates safely by requiring human intervention in manual mode(`enable_rtc` is true) at critical junctures and incorporating automatic safety checks in both modes to prevent unsafe operations in the presence of dynamic obstacles.

## Design

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/image-03-730b9b1716.svg)

## General parameters for start\_planner

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| th\_arrived\_distance | [m] | double | distance threshold for arrival of path termination | 1.0 |
| th\_stopped\_velocity | [m/s] | double | velocity threshold for arrival of path termination | 0.01 |
| th\_stopped\_time | [s] | double | time threshold for arrival of path termination | 1.0 |
| th\_distance\_to\_middle\_of\_the\_road | [m] | double | distance threshold to determine if the vehicle is on the middle of the road | 0.1 |
| collision\_check\_margins | [m] | double | Obstacle collision check margins list | [2.0, 1.0, 0.5, 0.1] |
| shift\_collision\_check\_distance\_from\_end | [m] | double | collision check distance from end shift end pose | -10.0 |
| geometric\_collision\_check\_distance\_from\_end | [m] | double | collision check distance from end geometric end pose | 0.0 |
| collision\_check\_margin\_from\_front\_object | [m] | double | collision check margin from front object | 5.0 |
| skip\_rear\_vehicle\_check | - | bool | flag to skip rear vehicle check (rear vehicle check is performed to skip safety check and proceed with departure when the ego vehicle is obstructing the progress of a rear vehicle) | false |
| extra\_width\_margin\_for\_rear\_obstacle | [m] | double | extra width that is added to the perceived rear obstacle's width when deciding if the rear obstacle can overtake the ego vehicle while it is merging to a lane from the shoulder lane | 0.5 |
| object\_types\_to\_check\_for\_path\_generation.check\_car | - | bool | flag to check car for path generation | true |
| object\_types\_to\_check\_for\_path\_generation.check\_truck | - | bool | flag to check truck for path generation | true |
| object\_types\_to\_check\_for\_path\_generation.check\_bus | - | bool | flag to check bus for path generation | true |
| object\_types\_to\_check\_for\_path\_generation.check\_bicycle | - | bool | flag to check bicycle for path generation | true |
| object\_types\_to\_check\_for\_path\_generation.check\_motorcycle | - | bool | flag to check motorcycle for path generation | true |
| object\_types\_to\_check\_for\_path\_generation.check\_pedestrian | - | bool | flag to check pedestrian for path generation | true |
| object\_types\_to\_check\_for\_path\_generation.check\_unknown | - | bool | flag to check unknown for path generation | true |
| center\_line\_path\_interval | [m] | double | reference center line path point interval | 1.0 |
| minimum\_shift\_length | [m] | double | the minimum required shift length for a clothoid path to be considered valid. | 0.5 |
| turn\_signal\_on\_centerline\_start | - | string | default turn signal direction when pulling out from centerline | RIGHT |

| clothoid\_initial\_velocity | [m/s] | double | initial velocity for clothoid path | 1.0 |
| clothoid\_acceleration | [m/s²] | double | acceleration for clothoid path | 1.0 |
| clothoid\_max\_steer\_angles\_deg | [deg] | double | maximum steer angles to try | [5.0, 10.0, 20.0] |
| clothoid\_max\_steer\_angle\_rate\_deg\_per\_sec | [deg/s] | double | maximum steer angle rate | 10.0 |
| check\_clothoid\_path\_lane\_departure | - | bool | flag to check if clothoid path footprints are out of lane | true |

### **Ego vehicle's velocity planning**

WIP

### **Safety check in free space area**

WIP

## Parameters for safety check

### Stop Condition Parameters

Parameters under `stop_condition` define the criteria for stopping conditions.

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| maximum\_deceleration\_for\_stop | [m/s^2] | double | Maximum deceleration allowed for a stop | 1.0 |
| maximum\_jerk\_for\_stop | [m/s^3] | double | Maximum jerk allowed for a stop | 1.0 |

### Ego Predicted Path Parameters

Parameters under `path_safety_check.ego_predicted_path` specify the ego vehicle's predicted path characteristics.

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| min\_velocity | [m/s] | double | Minimum velocity of the ego vehicle's predicted path | 0.0 |
| acceleration | [m/s^2] | double | Acceleration for the ego vehicle's predicted path | 1.0 |
| max\_velocity | [m/s] | double | Maximum velocity of the ego vehicle's predicted path | 1.0 |
| time\_horizon\_for\_front\_object | [s] | double | Time horizon for predicting front objects | 10.0 |
| time\_horizon\_for\_rear\_object | [s] | double | Time horizon for predicting rear objects | 10.0 |
| time\_resolution | [s] | double | Time resolution for the ego vehicle's predicted path | 0.5 |
| delay\_until\_departure | [s] | double | Delay until the ego vehicle departs | 1.0 |

### Target Object Filtering Parameters

Parameters under `target_filtering` are related to filtering target objects for safety check.

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| safety\_check\_time\_horizon | [s] | double | Time horizon for predicted paths of the ego and dynamic objects | 5.0 |
| safety\_check\_time\_resolution | [s] | double | Time resolution for predicted paths of the ego and dynamic objects | 1.0 |
| object\_check\_forward\_distance | [m] | double | Forward distance for object detection | 10.0 |
| object\_check\_backward\_distance | [m] | double | Backward distance for object detection | 100.0 |
| ignore\_object\_velocity\_threshold | [m/s] | double | Velocity threshold below which objects are ignored | 1.0 |
| object\_types\_to\_check.check\_car | - | bool | Flag to check cars | true |
| object\_types\_to\_check.check\_truck | - | bool | Flag to check trucks | true |
| object\_types\_to\_check.check\_bus | - | bool | Flag to check buses | true |
| object\_types\_to\_check.check\_trailer | - | bool | Flag to check trailers | true |
| object\_types\_to\_check.check\_bicycle | - | bool | Flag to check bicycles | true |
| object\_types\_to\_check.check\_motorcycle | - | bool | Flag to check motorcycles | true |
| object\_types\_to\_check.check\_pedestrian | - | bool | Flag to check pedestrians | true |
| object\_types\_to\_check.check\_unknown | - | bool | Flag to check unknown object types | false |
| object\_lane\_configuration.check\_current\_lane | - | bool | Flag to check the current lane | true |
| object\_lane\_configuration.check\_right\_side\_lane | - | bool | Flag to check the right side lane | true |
| object\_lane\_configuration.check\_left\_side\_lane | - | bool | Flag to check the left side lane | true |
| object\_lane\_configuration.check\_shoulder\_lane | - | bool | Flag to check the shoulder lane | true |
| object\_lane\_configuration.check\_other\_lane | - | bool | Flag to check other lanes | false |
| include\_opposite\_lane | - | bool | Flag to include the opposite lane in check | false |
| invert\_opposite\_lane | - | bool | Flag to invert the opposite lane check | false |
| check\_all\_predicted\_path | - | bool | Flag to check all predicted paths | true |
| use\_all\_predicted\_path | - | bool | Flag to use all predicted paths | true |
| use\_predicted\_path\_outside\_lanelet | - | bool | Flag to use predicted paths outside of lanelets | false |

### Safety Check Parameters

Parameters under `safety_check_params` define the configuration for safety check.

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| enable\_safety\_check | - | bool | Flag to enable safety check | true |
| check\_all\_predicted\_path | - | bool | Flag to check all predicted paths | true |
| publish\_debug\_marker | - | bool | Flag to publish debug markers | false |
| rss\_params.rear\_vehicle\_reaction\_time | [s] | double | Reaction time for rear vehicles | 2.0 |
| rss\_params.rear\_vehicle\_safety\_time\_margin | [s] | double | Safety time margin for rear vehicles | 1.0 |
| rss\_params.lateral\_distance\_max\_threshold | [m] | double | Maximum lateral distance threshold | 2.0 |
| rss\_params.longitudinal\_distance\_min\_threshold | [m] | double | Minimum longitudinal distance threshold | 3.0 |
| rss\_params.longitudinal\_velocity\_delta\_time | [s] | double | Delta time for longitudinal velocity | 0.8 |
| hysteresis\_factor\_expand\_rate | - | double | Rate to expand/shrink the hysteresis factor | 1.0 |
| collision\_check\_yaw\_diff\_threshold | - | double | Maximum yaw difference between ego and object when executing collision assessment | 1.578 |

## **Path Generation**

There are three path generation methods.

### **shift pull out**

This is the most basic method of starting path planning and is used on road lanes and shoulder lanes when there is no particular obstruction.

Pull out distance is calculated by the speed, lateral deviation, and the lateral jerk. The lateral jerk is searched for among the predetermined minimum and maximum values, and the one that generates a safe path is selected.

- Generate the road lane centerline and shift it to the current position.
- In the section between merge start and end, path is shifted by a method that is used to generate avoidance path (four segmental constant jerk polynomials)
- Combine this path with center line of road lane

![shift_pull_out](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/shift_pull_out.drawio.svg)

[shift pull out video](https://user-images.githubusercontent.com/39142679/187872468-6d5057ee-e039-499b-afc7-fe0dc8052a6b.mp4)

#### parameters for shift pull out

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| check\_shift\_path\_lane\_departure | [-] | bool | flag whether to check if shift path footprints are out of lane | true |
| allow\_check\_shift\_path\_lane\_departure\_override | [-] | bool | flag to override/cancel lane departure check if the ego vehicle's starting pose is already out of lane | false |
| shift\_pull\_out\_velocity | [m/s] | double | velocity of shift pull out | 2.0 |
| pull\_out\_sampling\_num | [-] | int | Number of samplings in the minimum to maximum range of lateral\_jerk | 4 |
| maximum\_lateral\_jerk | [m/s3] | double | maximum lateral jerk | 2.0 |
| minimum\_lateral\_jerk | [m/s3] | double | minimum lateral jerk | 0.1 |
| minimum\_shift\_pull\_out\_distance | [m] | double | minimum shift pull out distance. if calculated pull out distance is shorter than this, use this for path generation. | 0.0 |
| maximum\_curvature | [1/m] | double | maximum curvature. Calculate the required pull out distance from this maximum curvature, assuming the reference path is considered a straight line and shifted by two approximate arcs. This does not compensate for curvature in a shifted path or curve. | 0.07 |
| end\_pose\_curvature\_threshold | [1/m] | double | The curvature threshold which is used for calculating the shift pull out distance. The shift end pose is shifted forward so that curvature on shift end pose is less than this value. This is to prevent the generated path from having a large curvature when the end pose is on a curve. If a shift end pose with a curvature below the threshold is not found, the shift pull out distance is used as the distance to the point with the lowest curvature among the points beyond a certain distance. | 0.01 |

### **geometric pull out**

Generate two arc paths with discontinuous curvature. Ego-vehicle stops once in the middle of the path to control the steer on the spot.
See also [[1]](https://www.sciencedirect.com/science/article/pii/S1474667015347431) for details of the algorithm.

![geometric_pull_out](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/geometric_pull_out.drawio.svg)

[geometric pull out video](https://user-images.githubusercontent.com/39142679/181024707-3e7ca5ee-62de-4334-b9e9-ded313de1ea1.mp4)

#### parameters for geometric pull out

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| divide\_pull\_out\_path | [-] | bool | flag whether to divide arc paths. The path is assumed to be divided because the curvature is not continuous. But it requires a stop during the departure. | false |
| geometric\_pull\_out\_velocity | [m/s] | double | velocity of geometric pull out | 1.0 |
| lane\_departure\_margin | [m] | double | margin of deviation to lane right | 0.2 |
| lane\_departure\_check\_expansion\_margin | [m] | double | margin to expand the ego vehicle footprint when doing lane departure checks | 0.0 |
| geometric\_pull\_out\_max\_steer\_angle\_margin\_scale | [-] | double | scaling factor applied to the maximum steering angle (max\_steer\_angle) defined in vehicle\_info parameter | 0.72 |

### **clothoid pull out**

Generate smooth paths using clothoid curves that provide continuous curvature transitions. The clothoid path consists of three segments: entry clothoid, circular arc, and exit clothoid, ensuring smooth steering transitions.

![clothoid_pull_out](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/clothoid_pullout.drawio.svg)

#### Path Generation Flow

The path is generated with following flow.

1. **Parameter Setting and Initialization**
2. **Get lane information**

   - Get road lane (target lane) and shoulder lane (start lane) information.
3. **Start and Target Path Generation**

   - Generate the centerline path of the target lane.
   - Generated the straight path of the shoulder lane.
4. **Circular Arc Path Generation (red dotted line in the following figure)**

   - Generate circular arc path using the same method as geometric pull out.
   - Calculate composite arc path with two arc segments (entry and exit arcs).
   - This process is repeated with gradually increasing maximum steering angles from the parameter list `clothoid_max_steer_angles_deg` (e.g., [5.0, 10.0, 20.0] degrees) until a valid path is found.
5. **Clothoid Approximation**

   - Convert the circular arc segments to clothoid curves.
   - Generate three-segment clothoid path: entry clothoid → circular arc → exit clothoid for each arc path (**black dotted line in the following figure**).
   - Apply rigid transform to align the start and goal points of the approximated clothoid path with the original arc path (**black line in the following figure**).
6. **Lane Departure Check and Path Validation**
7. **Collision Check**

![clothoid_pullout_path_generation_flow](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/clothoid_flow.drawio.svg)

The following diagram illustrates the process flow for generating clothoid paths:

```
flowchart TD
    A[Start: Parameter Setting and Initialization] --> B[Get Lane Information]
    B --> C[Start and Target Path Generation]
    C --> D[Select a candidate max steer angle]
    D --> E[Generate Circular Arc Path]
    E --> F[Clothoid Approximation]
    F --> G[Lane Departure Check and Path Validation]
    G --> H[Collision Check]
    H --> I[Is the path valid?]
    I -->|No| D
    I -->|Yes| J[End: Return Path]
    D -->|No More Candidate| K[Path Generation Failure]
```

#### parameters for clothoid pull out

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| clothoid\_initial\_velocity | [m/s] | double | initial velocity for clothoid path | 1.0 |
| clothoid\_acceleration | [m/s²] | double | acceleration for clothoid path | 1.0 |
| clothoid\_max\_steer\_angles\_deg | [deg] | double | maximum steer angles to try | [5.0, 10.0, 20.0] |
| clothoid\_max\_steer\_angle\_rate\_deg\_per\_sec | [deg/s] | double | maximum steer angle rate | 10.0 |
| clothoid\_collision\_check\_distance\_from\_end | [m] | double | collision check distance from end for clothoid planner | 0.0 |
| check\_clothoid\_path\_lane\_departure | [-] | bool | flag whether to check if clothoid path footprints are out of lane | true |

#### Limitation

- **Violation of the max curvature**: During the rigid transformation of the approximated clothoid path to align with the original circular arc path, the scaling factor may cause the maximum steering angle constraint to be violated. When the scale factor is smaller than 1.0, the curvature of the transformed clothoid path may exceed the maximum curvature corresponding to the specified maximum steering angle.

- **Yaw Angle Deviation at Path Endpoints**: The rigid transformation process introduces yaw angle deviations at the start and end points of the clothoid path. As shown in the figure above, note that the yaw angles at the start and end points differ between the original circular arc path (red dotted line) and the transformed clothoid path (black solid line).

## **backward pull out start point search**

If a safe path cannot be generated from the current position, search backwards for a pull out start point at regular intervals(default: `2.0`).

![pull_out_after_back](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/images/pull_out_after_back.drawio.svg)

[pull out after backward driving video](https://user-images.githubusercontent.com/39142679/181025149-8fb9fb51-9b8f-45c4-af75-27572f4fba78.mp4)

### **parameters for backward pull out start point search**

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| enable\_back | [-] | bool | flag whether to search backward for start\_point | true |
| search\_priority | [-] | string[] | list of planner types in priority order. Available: "SHIFT", "GEOMETRIC", "CLOTHOID" | ["SHIFT","GEOMETRIC"] |
| search\_policy | [-] | string | search policy: "planner\_priority" (planner-first: SHIFT all candidates, then GEOMETRIC ...) or "distance\_priority" (candidate-first: 0m SHIFT, 0m GEOMETRIC, 2m SHIFT, ...) | "planner\_priority" |
| max\_back\_distance | [m] | double | maximum back distance | 30.0 |
| backward\_search\_resolution | [m] | double | distance interval for searching backward pull out start point | 2.0 |
| backward\_path\_update\_duration | [s] | double | time interval for searching backward pull out start point. this prevents chattering between back driving and pull\_out | 3.0 |
| ignore\_distance\_from\_lane\_end | [m] | double | If distance from shift start pose to end of shoulder lane is less than this value, this start pose candidate is ignored | 15.0 |

### **freespace pull out**

If the vehicle gets stuck with pull out along lanes, execute freespace pull out.
To run this feature, you need to set `parking_lot` to the map, `activate_by_scenario` of [costmap\_generator](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_costmap_generator/) to `false` and `enable_freespace_planner` to `true`

![](https://user-images.githubusercontent.com/39142679/270964106-ae688bca-1709-4e06-98c4-90f671bb8246.png)

#### Unimplemented parts / limitations for freespace pull out

- When a short path is generated, the ego does can not drive with it.
- Complex cases take longer to generate or fail.
- The drivable area is not guaranteed to fit in the parking\_lot.

#### Parameters freespace parking

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| enable\_freespace\_planner | [-] | bool | this flag activates a free space pullout that is executed when a vehicle is stuck due to obstacles in the lanes where the ego is located | true |
| end\_pose\_search\_start\_distance | [m] | double | distance from ego to the start point of the search for the end point in the freespace\_pull\_out driving lane | 20.0 |
| end\_pose\_search\_end\_distance | [m] | double | distance from ego to the end point of the search for the end point in the freespace\_pull\_out driving lane | 30.0 |
| end\_pose\_search\_interval | [m] | bool | interval to search for the end point in the freespace\_pull\_out driving lane | 2.0 |

See [freespace\_planner](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_freespace_planner/) for other parameters.
