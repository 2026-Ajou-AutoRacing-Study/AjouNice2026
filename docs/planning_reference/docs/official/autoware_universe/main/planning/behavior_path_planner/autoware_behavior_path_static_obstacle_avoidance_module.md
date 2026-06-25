---
source_type: rendered_autoware_docs
site: autoware_universe
title: Avoidance module for static objects
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/
rendered_html: rendered/autoware_universe_html/index-203.html
generated: 2026-06-24T14:59:34.750385+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Avoidance module for static objects

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/purpose/rviz.png)

## Purpose/Role

This is a rule-based avoidance module, which runs based on perception output data, HDMap, current path and route. This module is designed to create avoidance paths for static (=stopped) objects in simple situations. Currently, this module doesn't support dynamic (=moving) objects.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/purpose/avoidance.png)

This module has an [RTC interface](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_rtc_interface/), and the user can select operation mode from MANUAL/AUTO depending on the performance of the vehicle's sensors. If the user selects MANUAL mode, this module outputs a candidate avoidance path and awaits operator approval. In the case where the sensor/perception performance is insufficient and false positives occur, we recommend MANUAL mode to prevent unnecessary avoidance maneuvers.

If the user selects AUTO mode, this module modifies the current following path without operator approval. If the sensor/perception performance is sufficient, use AUTO mode.

### Limitations

This module allows developers to design vehicle behavior in avoidance planning using specific rules. Due to the property of rule-based planning, the algorithm cannot compensate for not colliding with obstacles in complex cases. This is a trade-off between "be intuitive and easy to design" and "be hard to tune but can handle many cases". This module adopts the former policy and therefore this output should be checked more strictly in the later stage. In the .iv reference implementation, there is another avoidance module in the motion planning module that uses optimization to handle the avoidance in complex cases. (Note that, the motion planner needs to be adjusted so that the behavior result will not be changed much in the simple case and this is a typical challenge for the behavior-motion hierarchical architecture.)

### Why is avoidance in behavior module?

This module executes avoidance over lanes, and the decision requires the lane structure information to take care of traffic rules (e.g. it needs to send an indicator signal when the vehicle crosses a lane). The difference between the motion and behavior modules in the planning stack is whether the planner takes traffic rules into account, which is why this avoidance module exists in the behavior module.

If you would like to know the overview rather than the detail, please skip the next section and refer to [FAQ](#frequently-asked-questions).

## Inner workings/Algorithms

This module mainly has two parts, target filtering and path generation. At first, all objects are filtered by several conditions. In this step, the module checks avoidance feasibility and necessity. After that, this module generates avoidance path outline, which we call **shift line**, based on filtered objects. The shift lines are set into [path shifter](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_path_generation_design/), which is a library for path generation, to create a smooth shift path. Additionally, this module has a feature to check non-target objects so that the ego can avoid target objects safely. This feature receives a generated avoidance path and surrounding objects, and judges the current situation. Lastly, this module updates current ego behavior.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-01-1decb4596f.svg)

## Target object filtering

### Overview

The module uses the following conditions to filter avoidance target objects.

| Check condition | Target class | Details | If conditions are not met |
| --- | --- | --- | --- |
| Is an avoidance target class object? | All | Use can select avoidance target class from config file. | Never avoid it. |
| Is a stopped object? | All | Objects keep higher speed than `th_moving_speed` for longer period of time than `th_moving_time` is judged as moving. | Never avoid it. |
| Is within detection area? | All | The module creates detection area to filter target objects roughly based on lateral margin in config file. (see [here](#width-of-detection-area)) | Never avoid it. |
| Isn't there enough lateral distance between the object and path? | All | - | Never avoid it. |
| Is near the centerline of ego lane? | All | - | It depends on other conditions. |
| Is there a crosswalk near the object? | Pedestrian, Bicycle | The module doesn't avoid the Pedestrian and Bicycle nearer the crosswalk because the ego should stop in front of it if they're crossing the road. (see [here](#for-crosswalk-users)) | Never avoid it. |
| Is the distance between the object and traffic light along the path longer than the threshold? | Car, Truck, Bus, Trailer | The module uses this condition when there is ambiguity about whether the vehicle is parked. | It depends on other conditions. |
| Is the distance between the object and crosswalk light along the path longer than threshold? | Car, Truck, Bus, Trailer | Same as above. | It depends on other conditions. |
| Is the stopping time longer than threshold? | Car, Truck, Bus, Trailer | Same as above. | It depends on other conditions. |
| Is within intersection? | Car, Truck, Bus, Trailer | The module assumes that there isn't any parked vehicle within intersection. | It depends on other conditions. |
| Is on ego lane? | Car, Truck, Bus, Trailer | - | It depends on other conditions. |
| Is a parked vehicle? | Car, Truck, Bus, Trailer | The module judges whether the vehicle is a parked vehicle based on its lateral offset. (see [here](#judge-if-its-a-parked-vehicle)) | It depends on other conditions. |
| Is merging into ego lane from other lane? | Car, Truck, Bus, Trailer | The module judges the vehicle behavior based on its yaw angle and offset direction. (see [here](#judge-vehicle-behavior)) | It depends on other conditions. |
| Is merging into other lane from ego lane? | Car, Truck, Bus, Trailer | Same as above. | It depends on other conditions. |

### Common conditions

#### Detection area

The module generates detection area for target filtering based on the following parameters:

```
      # avoidance is performed for the object type with true
      target_object:
      ...
          lateral_margin:
            soft_margin: 0.3                            # [m]
            hard_margin: 0.2                            # [m]
            hard_margin_for_parked_vehicle: 0.7         # [m]
      ...
      # For target object filtering
      target_filtering:
      ...
        # detection area generation parameters
        detection_area:
          static: false                                 # [-]
          min_forward_distance: 50.0                    # [m]
          max_forward_distance: 150.0                   # [m]
          backward_distance: 10.0                       # [m]
```

##### Width of detection area

1. Get the largest lateral margin of all classes (Car, Truck, ...). The margin is the sum of `soft_margin` and `hard_margin_for_parked_vehicle`.
2. The detection area width is the sum of ego vehicle width and the largest lateral margin.

##### Longitudinal distance of detection area

If the parameter `detection_area.static` is set to `true`, the module creates detection area whose longitudinal distance is `max_forward_distance`.

If the parameter `detection_area.static` is set to `false`, the module creates a detection area so that the ego can avoid objects with minimum lateral jerk value. Thus, the longitudinal distance depends on maximum lateral shift length, lateral jerk constraints and current ego speed. Additionally, it has to consider the distance used for the preparation phase.

```c++
...
const auto max\_shift\_length = std::max(
std::abs(parameters\_->max\_right\_shift\_length), std::abs(parameters\_->max\_left\_shift\_length));
const auto dynamic\_distance =
PathShifter::calcLongitudinalDistFromJerk(max\_shift\_length, getLateralMinJerkLimit(), speed);

```
return std::clamp(
  1.5 * dynamic_distance + getNominalPrepareDistance(),
  parameters_->object_check_min_forward_distance,
  parameters_->object_check_max_forward_distance);
```

```

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/target_filter/detection_area.svg)

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/target_filter/detection_area_rviz.png)

### Conditions for non-vehicle type objects

#### For crosswalk users

If Pedestrian and Bicycle are closer to crosswalk than threshold 2.0m (hard coded for now), the module judges they're crossing the road and never avoids them.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/target_filter/crosswalk.svg)

### Conditions for vehicle type objects

#### Judge vehicle behavior

The module classifies vehicles into the following three behaviors based on yaw angle and offset direction.

```yaml

# params for filtering objects that are in intersection

intersection:
yaw\_deviation: 0.349 # [rad] (default 20.0deg)
```

| Behavior | Details | Figure |
| --- | --- | --- |
| NONE | If the object's relative yaw angle to lane is less than threshold `yaw_deviation`, it is classified into `NONE`. | fig |
| MERGING | See following flowchart. | fig |
| DEVIATING | See following flowchart. | fig |

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-02-003327de53.svg)

#### Judge if it's a parked vehicle

Not only the length from the centerline, but also the length from the road shoulder is calculated and used for the filtering process. In this logic, it calculates ratio of **actual shift length** to **shiftable shift length** as follows. If the result is larger than threshold `th_shiftable_ratio`, the module judges the vehicle is a parked vehicle.

\[
L\_{d} = \frac{W\_{lane} - W\_{obj}}{2}, \\
ratio = \frac{L\_{a}}{L\_{d}}
\]

- \(L\_{d}\) : shiftable length.
- \(L\_{a}\) : actual shift length.
- \(W\_{lane}\) : lane width.
- \(W\_{obj}\) : object width.

![fig2](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/target_filter/parked_vehicle.svg)

### Target object filtering

| Situation | Details | Ego behavior |
| --- | --- | --- |
| Vehicle is within intersection area defined in HDMap. The module ignores vehicles following a lane or merging into ego lane. | fig | Never avoid it. |
| Vehicle is on ego lane. There are adjacent lanes for both sides. | fig | Never avoid it. |
| Vehicle is merging into other lane from ego lane. Most of its footprint is on ego lane. | fig | Never avoid it. |
| Vehicle is merging into ego lane from other lane. Most of its footprint is on ego lane. | fig | Never avoid it. |
| Vehicle does not appear to be parked and is stopped in front of a crosswalk or traffic light. | fig | Never avoid it. |
| Vehicle stops on ego lane while pulling over to the side of the road. | fig | Avoid it immediately. |
| Vehicle stops on adjacent lane. | fig | Avoid it immediately. |
| Vehicle stops on ego lane without pulling over to the side of the road. | fig | Set the parameter `avoidance_for_ambiguous_vehicle.enable` to `true`, the module avoids ambiguous vehicle. |
| Vehicle is merging into ego lane from other lane. | fig | Set the parameter `avoidance_for_ambiguous_vehicle.enable` to `true`, the module avoids ambiguous vehicle. |
| Vehicle is merging into other lane from ego lane. | fig | Set the parameter `avoidance_for_ambiguous_vehicle.enable` to `true`, the module avoids ambiguous vehicle. |

### Flowchart

There are three main filtering functions `isSatisfiedWithCommonCondition()`, `isSatisfiedWithVehicleCondition()` and `isSatisfiedWithNonVehicleCondition()`. The filtering process is executed according to the following flowchart. Additionally, the module checks avoidance necessity in `isNoNeedAvoidanceBehavior()` based on the object pose, ego path and lateral margin in the config file.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-03-f31db6e4da.svg)

#### Common conditions

At first, the function `isSatisfiedWithCommonCondition()` includes conditions used for all object classes.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-04-9380d26a67.svg)

#### Conditions for vehicle

Target class:

- Car
- Truck
- Bus
- Trailer

As a next step, the object is filtered by a condition specialized for its class.

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-05-6aba7573da.svg)

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-06-79f40c7965.svg)

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-07-c4a22baac0.svg)

#### Conditions for non-vehicle objects

- Pedestrian
- Bicycle

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-08-305b69ee18.svg)

## When target object has gone

User can select the ego behavior when the target object has gone.

```
cancel:
  enable: true # [-]
```

If the above parameter is `true`, this module reverts avoidance path when the following conditions are met.

- All target objects have gone.
- The ego vehicle hasn't initiated avoidance maneuver yet.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/cancel/cancel.png)

If the parameter is `false`, this module keeps running even after the target object has gone.

## Path generation

### How to prevent shift line chattering that is caused by perception noise

Since the object recognition result contains noise related to position, orientation and polygon shape, if the module uses the raw object recognition results in path generation, the output path will be directly affected by the noise. Therefore, in order to reduce the influence of the noise, this module generates a polygon for each target object, and the output path is generated based on that.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/envelope_polygon_rviz.png)

The envelope polygon is a rectangle box, whose size depends on the object's polygon and buffer parameter `envelope_buffer_margin`. Additionally, it is always parallel to the reference path. When the module finds a target object for the first time, it initializes the polygon.

```
        car:
          ...
          envelope_buffer_margin: 0.5                   # [m] FOR DEVELOPER
```

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/envelope_polygon.png)

The module creates a one-shot envelope polygon by using the latest object pose and raw polygon in every planning cycle. On the other hand, the module uses the envelope polygon information created in the last planning cycle in order to update the envelope polygon with consideration of the pose covariance.

If the pose covariance is smaller than the threshold, the envelope polygon would be updated according to the following logic. If the one-shot envelope polygon is not within the previous envelope polygon, the module creates a new envelope polygon. Otherwise, it keeps the previous envelope polygon.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/polygon_update.png)

When the pose covariance is larger than the threshold, it is compared with the maximum pose covariance of each object. If the value is smaller, the one-shot envelope polygon is used directly as the envelope polygon. Otherwise, it keeps the previous envelope polygon.

By doing this process, the envelope polygon size and pose will converge even if perception output includes noise in object pose or shape.

### Relationship between envelope polygon and avoidance path

The avoidance path has two shift sections, whose start or end point position depends on the envelope polygon. The end point of the avoidance shift section and start point of the return shift section are fixed based on the envelope polygon and the other side edges are dynamically changed based on ego speed, shift length, lateral jerk constraints, etc.

The lateral positions of the two points are decided so that there can be enough space (=lateral margin) between ego body and the most overhang point of the envelope polygon edge points. User can adjust lateral margin with the following parameters.

```
        car:
          ...
          lateral_margin:
            soft_margin: 0.3                            # [m]
            hard_margin: 0.2                            # [m]
            hard_margin_for_parked_vehicle: 0.7         # [m]
```

The longitudinal positions depends on the envelope polygon, ego vehicle specification and the following parameters. The longitudinal distance between avoidance shift section end point and envelope polygon (=front longitudinal buffer) is the sum of `front_overhang` defined in `vehicle_info.param.yaml` and `longitudinal_margin` if the parameter `consider_front_overhang` is `true`. If `consider_front_overhang` is `false`, only `longitudinal_margin` is considered. Similarly, the distance between the return shift section start point and envelope polygon (=rear longitudinal buffer) is the sum of `rear_overhang` and `longitudinal_margin`.

```yaml

```
  target_object:
    car:
      ...
      longitudinal_margin: 0.0                      # [m]

  ...
  avoidance:
    ...
    longitudinal:
      ...
      consider_front_overhang: true                 # [-]
      consider_rear_overhang: true                  # [-]
```

```

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/margin.png)

### Lateral margin

As mentioned above, user can adjust lateral margin by changing the following two types of parameters. The `soft_margin` is a soft constraint parameter for lateral margin. The `hard_margin` and `hard_margin_for_parked_vehicle` are hard constraint parameters.

```
        car:
          ...
          lateral_margin:
            soft_margin: 0.3                            # [m]
            hard_margin: 0.2                            # [m]
            hard_margin_for_parked_vehicle: 0.7         # [m]
```

Basically, this module tries to generate an avoidance path in order to keep lateral distance, which is the sum of `soft_margin` and `hard_margin`/`hard_margin_for_parked_vehicle`, from the avoidance target object.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/soft_hard.png)

But if there isn't enough space to keep `soft_margin` distance, this module shortens soft constraint lateral margin. The parameter `soft_margin` is a maximum value of soft constraint, and actual soft margin can be a value between 0.0 and `soft_margin`. On the other hand, this module definitely keeps `hard_margin` or `hard_margin_for_parked_vehicle` depending on the situation. Thus, the minimum value of total lateral margin is `hard_margin`/`hard_margin_for_parked_vehicle`, and the maximum value is the sum of `hard_margin`/`hard_margin_for_parked_vehicle` and `soft_margin`.

The following figure shows the situation where this module shortens lateral soft constraint in order not to drive in the opposite lane when user sets parameter `use_lane_type` to `same_direction_lane`.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/adjust_margin.png)

This module avoids not only parked vehicles but also non-parked vehicles that stop temporarily for some reason (e.g. waiting for traffic light to change from red to green). Additionally, this module has two types of hard margin parameters, `hard_margin` and `hard_margin_for_parked_vehicle` and judges if it is a parked vehicle or not for each vehicle because it takes the risk of vehicle doors opening suddenly and people getting out from parked vehicles into consideration.

Users should set `hard_margin_for_parked_vehicle` larger than `hard_margin` to prevent collisions with doors or people who suddenly exit a vehicle.

This module has only one parameter `soft_margin` for soft lateral margin constraint.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/hard_margin.png)

As the hard margin parameters define the distance the user definitely wants to maintain, they are used in the logic to check whether the ego can pass the side of the target object without executing an avoidance maneuver as well.

If the lateral distance is less than `hard_margin`/`hard_margin_for_parked_vehicle` when assuming that the ego follows the current lane without an avoidance maneuver, this module thinks the ego can not pass the side of the object safely and the ego must avoid it. In this case, this module inserts a stop point until the avoidance maneuver is allowed to execute so that the ego can avoid the object after approval. (For example, the ego keeps stopping in front of such an object until the operator approves the avoidance maneuver if the module is in MANUAL mode.)

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/must_avoid.png)

On the other hand, if the lateral distance is larger than `hard_margin`/`hard_margin_for_parked_vehicle`, this module doesn't insert a stop point even when it is waiting for approval because it thinks it is possible to pass the side of the object safely.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/pass_through.png)

### When there is not enough space

This module inserts a stop point only when the ego can potentially avoid the object. So, if it is not able to keep a distance more than `hard_margin`/`hard_margin_for_parked_vehicle`, this module does nothing. The following figure shows the situation where this module is not able to keep enough lateral distance when the user sets parameter `use_lane_type` to `same_direction_lane`.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/do_nothing.png)

Info

In this situation, the obstacle stop feature in [obstacle\_cruise\_module](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/) is responsible for ego vehicle safety.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/insufficient_drivable_space.png)

### Shift length calculation

The lateral shift length is the sum of `overhang_distance`, lateral margin, whose value is set in the config file, and half of ego vehicle width defined in `vehicle_info.param.yaml`. On the other hand, the module limits the shift length depending on the space the module can use for an avoidance maneuver and the parameters `soft_drivable_bound_margin` `hard_drivable_bound_margin`. Basically, the shift length is limited so that the ego doesn't get closer than `soft_drivable_bound_margin` to the drivable boundary. But the module allows the threshold to be relaxed from `soft_drivable_bound_margin` to `hard_drivable_bound_margin` when the road is narrow.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/lateral.png)

Usable lanes for the avoidance module can be selected using the config file.

```
      ...
      # drivable lane setting. This module is able to use not only current lane but also right/left lane
      # if the current lane(=lanelet::Lanelet) and the right/left lane share the boundary(=lanelet::Linestring) in HDMap.
      # "current_lane"           : use only current lane. This module doesn't use adjacent lane to avoid object.
      # "same_direction_lane"    : this module uses same direction lane to avoid object if needed.
      # "opposite_direction_lane": this module uses both same direction and opposite direction lanes.
      use_lane_type: "opposite_direction_lane"
```

When user set parameter `use_lane_type` to `opposite_direction_lane`, it is possible to use opposite lane.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/opposite_direction.png)

When user set parameter `use_lane_type` to `same_direction_lane`, the module doesn't create path that overlaps opposite lane.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/same_direction.png)

### Shift line generation

As mentioned above, the end point of the avoidance shift path and the start point of the return shift path, which are FIXED points, are calculated from envelope polygon. As a next step, the module adjusts the other side points depending on shift length, current ego speed and lateral jerk constrain params defined in the config file.

Since the two points are always on the centerline of the ego lane, the module only calculates longitudinal distance between the shift start and end point based on the following function. This function is defined in the path shifter library. See [this](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_path_generation_design/) page as well.

```
double PathShifter::calcLongitudinalDistFromJerk(
  const double lateral, const double jerk, const double velocity)
{
  const double j = std::abs(jerk);
  const double l = std::abs(lateral);
  const double v = std::abs(velocity);
  if (j < 1.0e-8) {
    return 1.0e10;
  }
  return 4.0 * std::pow(0.5 * l / j, 1.0 / 3.0) * v;
}
```

We call the line that connects shift start and end point `shift_line`, which the avoidance path is generated from with spline completion.

The start point of avoidance has another longitudinal constraint. In order to keep turning on the blinker for a few seconds before starting the avoidance maneuver, the avoidance start point must be further than the value (we call the distance `prepare_length`.) depending on ego speed from ego position.

```
longitudinal:
  min_prepare_time: 1.0 # [s]
  max_prepare_time: 2.0 # [s]
  min_prepare_distance: 1.0 # [m]
```

The `prepare_length` is calculated as the product of ego speed and `max_prepare_time`. (When the ego speed is zero, `min_prepare_distance` is used.)

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/path_generation/shift_line.png)

## Planning at RED traffic light

This module takes traffic light information into account so that the ego can behave properly. Sometimes, the ego straddles the lane boundary but we want to prevent the ego from stopping in front of a red traffic signal in such a situation. This is because the ego will block adjacent lanes and it is inconvenient for other vehicles.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/traffic_light/traffic_light.png)

So, this module controls shift length and shift start/end point in order to prevent the above situation.

### Control shift length

At first, if the ego hasn't initiated an avoidance maneuver yet, this module limits maximum shift length and uses **ONLY** current lane during a red traffic signal. This prevents the ego from blocking other vehicles even if this module executes the avoidance maneuver and the ego is caught by a red traffic signal.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/traffic_light/limit_shift_length.png)

### Control avoidance shift start point

Additionally, if the target object is farther than the stop line of the traffic light, this module sets the avoidance shift start point on the stop line in order to prevent the ego from stopping at a red traffic signal in the middle of an avoidance maneuver.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/traffic_light/shift_from_current_pos.png)
![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/traffic_light/shift_from_stop_line.png)

### Control return shift end point

If the ego has already initiated an avoidance maneuver, this module tries to set the return shift end point on the stop line.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/traffic_light/return_after_stop_line.png)
![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/traffic_light/return_before_stop_line.png)

## Safety check

This feature can be enabled by setting the following parameter to `true`.

```
      safety_check:
        ...
        enable: true                                    # [-]
```

This module pays attention not only to avoidance target objects but also non-target objects that are near the avoidance path, and if the avoidance path is unsafe due to surrounding objects, it reverts the avoidance maneuver and yields the lane to them.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/safety_check/safety_check_flow.png)

### Yield maneuver

Additionally, this module basically inserts a stop point in front of an avoidance target during yielding maneuvers in order to keep enough distance to avoid the target when it is safe to do so. If the shift side lane is congested, the ego stops at a point and waits.

This feature can be enabled by setting the following parameter to `true`.

```
yield:
  enable: true # [-]
```

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/safety_check/stop.png)

But if the lateral margin is larger than `hard_margin` (or `hard_margin_for_parked_vehicle`), this module doesn't insert a stop point because the ego can pass the side of the object safely without an avoidance maneuver.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/safety_check/not_stop.png)

### Safety check target lane

User can select the safety check area with the following parameters. Basically, we recommend the following configuration to check only the shift side lane. If users want to confirm safety strictly, please set `check_current_lane` and/or `check_other_side_lane` to `true`.

```
      safety_check:
      ...
        check_current_lane: false                       # [-]
        check_shift_side_lane: true                     # [-]
        check_other_side_lane: false                    # [-]
```

In the avoidance module, the function `path_safety_checker::isCentroidWithinLanelet` is used for filtering objects by lane.

```
bool isCentroidWithinLanelet(const PredictedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d object_centroid(object_pos.x, object_pos.y);
  return boost::geometry::within(object_centroid, lanelet.polygon2d().basicPolygon());
}
```

Info

If `check_current_lane` and/or `check_other_side_lane` are set to `true`, the possibility of false positives and unnecessary yield maneuvers increase.

### Safety check algorithm

This module uses common safety check logic implemented in `path_safety_checker` library. See [this](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_safety_check/) page.

### Limitation

#### Limitation-1

The current behavior when the module judges it is unsafe is so conservative because it is difficult to achieve aggressive maneuvers (e.g. increase speed in order to increase the distance from rear vehicle) for current planning architecture.

#### Limitation-2

The yield maneuver is executed **ONLY** when the vehicle has **NOT** initiated avoidance maneuver yet. (This module checks objects in the opposite lane but it is necessary to find very far objects to judge whether it is unsafe before avoidance maneuver.) If it detects a vehicle which is approaching the ego during an avoidance maneuver, this module doesn't revert the path or insert a stop point. For now, there is no feature to deal with this situation in this module. Thus, a new module is needed to adjust the path to avoid moving objects, or an operator must override.

Info

This module has a threshold parameter `th_avoid_execution` for the shift length, and judges that the vehicle is initiating avoidance if the vehicle current shift exceeds the value.

## Other features

### Compensation for detection lost

In order to prevent chattering of recognition results, once an obstacle is targeted, it is held for a while even if it disappears. This is effective when recognition is unstable. However, since it will result in over-detection (increases number of false-positives), it is necessary to adjust parameters according to the recognition accuracy (if `object_last_seen_threshold = 0.0`, the recognition result is 100% trusted).

### Drivable area expansion

This module supports drivable area expansion for following polygons defined in HDMap.

- Intersection area
- Hatched road marking
- Freespace area

Please set the flags to `true` when user wants to make it possible to use those areas in avoidance maneuvers.

```yaml

# drivable lane setting. This module is able to use not only current lane but also right/left lane

# if the current lane(=lanelet::Lanelet) and the right/left lane share the boundary(=lanelet::Linestring) in HDMap.

# "current\_lane" : use only current lane. This module doesn't use adjacent lane to avoid object.

# "same\_direction\_lane" : this module uses the same direction lane to avoid object if needed.

# "opposite\_direction\_lane": this module uses both same direction and opposite direction lanes.

use\_lane\_type: "opposite\_direction\_lane"

# drivable area setting

use\_intersection\_areas: true
use\_hatched\_road\_markings: true
use\_freespace\_areas: true
```

|  |  |  |
| --- | --- | --- |
| use\_lane\_type: same\_direction\_lane | fig |  |
| use\_lane\_type: opposite\_direction\_lane | fig |  |
| intersection area | fig | The intersection area is defined on Lanelet map. See [here](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md) |
| hatched road markings | fig | The hatched road marking is defined on Lanelet map. See [here](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md#hatched-road-markings-area) |
| freespace area | fig | The freespace area is defined on Lanelet map. (unstable) |

## Future extensions/Unimplemented parts

- **Consideration of the speed of the avoidance target**
  - In the current implementation, only stopped vehicle is targeted as an avoidance target. It is needed to support the overtaking function for low-speed vehicles, such as a bicycle. (It is actually possible to overtake the low-speed objects by changing the parameter, but the logic is not supported and thus the safety cannot be guaranteed.)
  - Overtaking (e.g., to overtake a vehicle running in front at 20 km/h at 40 km/h) may need to be handled outside the avoidance module. It should be discussed which module should handle it.

- **Cancel avoidance when target disappears**
  - In the current implementation, even if the avoidance target disappears, the avoidance path will remain. If there is no longer a need to avoid, it must be canceled.

- **Improved performance of avoidance target selection**
  - Essentially, avoidance targets are judged based on whether they are static objects or not. For example, a vehicle waiting at a traffic light should not be avoided because we know that it will start moving in the future. However this decision cannot be made in the current Autoware due to the lack of perception functions. Therefore, the current avoidance module limits the avoidance target to vehicles parked on the shoulder of the road, and executes avoidance only for vehicles that are stopped away from the center of the lane. However, this logic cannot avoid a vehicle that has broken down and is stopped in the center of the lane, which should be recognized as a static object by the perception module. There is room for improvement in the performance of this decision.

- **Resampling path**
  - Now the rough resolution resampling is processed to the output path in order to reduce the computational cost for the later modules. This resolution is set to a uniformly large value (e.g. `5m`), but a small resolution should be applied for complex paths.

## Debug

### Show `RCLCPP_DEBUG` on console

All debug messages are logged in the following namespaces.

- `planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.static_obstacle_avoidance` or,
- `planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.static_obstacle_avoidance.utils`

User can see debug information with the following command.

```
ros2 service call /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/config_logger logging_demo/srv/ConfigLogger "{logger_name: 'planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.static_obstacle_avoidance', level: DEBUG}"
```

### Visualize debug markers

User can enable publishing of debug markers with the following parameters.

```
debug:
  enable_other_objects_marker: false
  enable_other_objects_info: false
  enable_detection_area_marker: false
  enable_drivable_bound_marker: false
  enable_safety_check_marker: false
  enable_shift_line_marker: false
  enable_lane_marker: false
  enable_misc_marker: false
```

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/debug/debug_marker_rviz.png)

### Echoing debug message to find out why the objects were ignored

If for some reason, no shift point is generated for your object, you can check for the failure reason via `ros2 topic echo`.

![avoidance_debug_message_array](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/avoidance_debug_message_array.png)

To print the debug message, just run the following

```
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array
```

## Frequently asked questions

### Target objects

#### Does it avoid static objects and dynamic objects?

This module avoids static (stopped) objects and does not support dynamic (moving) objects avoidance. Dynamic objects are handled within the [dynamic obstacle avoidance module](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/).

#### What type (class) of object would it avoid?

It avoids cars, trucks, buses, trailers, bicycles, motorcycles, pedestrians, and unknown objects by default. Details are in the [Target object filtering section](#target-object-filtering).
The above objects are divided into vehicle type objects and non-vehicle type objects; the target object filtering would differ for vehicle types and non-vehicle types.

- Vehicle type objects: Car, Truck, Bus, Trailer, Motorcycle
- Non-vehicle type objects: Pedestrian, Bicycle

#### How does it judge if it is a target object or not?

The conditions for vehicle type objects and non-vehicle type objects are different. However, the main idea is that static objects on road shoulders within the planned path would be avoided.
Below are some examples when an avoidance path is generated for vehicle type objects.

- Vehicle stopping on ego lane while pulling over to the side of the road
- Vehicle stopping on adjacent lane

For more details refer to [vehicle type object](#conditions-for-vehicle-type-objects) and [non-vehicle object](#conditions-for-non-vehicle-type-objects).

#### What is an ambiguous target?

An ambiguous target refers to objects that may not be clearly identifiable as avoidance target due to limitations of current Autoware (ex. parked vehicle in the center of a lane).
This module will avoid clearly defined static objects automatically, whereas ambiguous targets would need some operator intervention.

#### How can I visualize the target object?

Target objects can be visualized using RViz, where the module's outputs, such as detected obstacles and planned avoidance paths, are displayed. For further information refer to the [debug marker section](#visualize-debug-markers).

#### How can I check the lateral distance to an obstacle?

Currently, there isn't any topic that outputs the relative position with the ego vehicle and target object.
Visual confirmation on RViz would be the only solution for now.

#### Does it avoid multiple objects at once?

Yes, the module is capable of avoiding multiple static objects simultaneously.
It generates multiple shift lines and calculates an avoidance path that navigates around each object.
Details are explained in the [How to decide path shape section](#multiple-obstacle-case-one-direction).

### Area used when avoiding

#### Which lanes are used to avoid objects?

This module is able to use not only the current lane but also adjacent lanes and opposite lanes. Usable lanes can be selected by the configuration file as noted in the [shift length calculation section](#shift-length-calculation).
It is assumed that there are no parked vehicles on the central lane in a situation where there are lanes on the left and right.

#### Would it avoid objects inside intersections?

Basically, the module assumes that there are no parked vehicles within intersections. Vehicles that follow the lane or merge into ego lane are non-target objects.
Vehicles waiting to make a right/left turn within an intersection can be avoided by expanding the drivable area in the configuration file, as noted in the [drivable area expansion section](#drivable-area-expansion).

#### Does it generate avoidance paths for any road type?

Drivable area can be expanded in the configuration file, as noted in the [drivable area expansion section](#drivable-area-expansion).

### Path generation

#### How is the avoidance path generated?

The avoidance path is generated by modifying the current reference path to navigate around detected static objects.
This is done using a rule-based shift line approach that ensures the vehicle remains within safe boundaries and follows the road while avoiding obstacles.
Details are explained in the [appendix](#appendix-shift-line-generation-pipeline).

#### Which way (right or left) is the avoidance path generated?

The behavior of avoiding depends on the target vehicle's center of gravity.
If the target object is on the left side of the ego lane the avoidance would be generated on the right side.
Currently, avoiding left-shifted obstacles from the left side is not supported (same for right-shifted objects).

#### Why is an envelope polygon used for the target object?

It is employed to reduce the influence of the perception/tracking noise for each target object.
The envelope polygon is a rectangle, whose size depends on the object's polygon and buffer parameter and it is always parallel to the reference path.
The envelope polygon is created by using the latest one-shot envelope polygon and the previous envelope polygon.
Details are explained in [How to prevent shift line chattering that is caused by perception noise section](#how-to-prevent-shift-line-chattering-that-is-caused-by-perception-noise).

#### What happens if the module cannot find a safe avoidance path?

If the module cannot find a safe avoidance path, the vehicle may stop or continue along its current path without performing an avoidance maneuver.
If there is a target object and there is enough space to avoid, the ego vehicle would stop at a position where an avoidance path could be generated; this is called the [yield manuever](#yield-maneuver).
On the other hand, where there is not enough space, this module has nothing to do and the [obstacle cruise module](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/) would be in charge of the object.

#### There seems to be an avoidance path, but the vehicle stops. What is happening?

This situation occurs when the module is operating in AUTO mode and the target object is ambiguous or when operating in MANUAL mode.
The generated avoidance path is presented as a candidate and requires operator approval before execution.
If the operator does not approve the path the ego vehicle would stop where it is possible to generate an avoidance path.

### Operation

#### What are the benefits of using MANUAL mode over AUTO mode?

MANUAL mode allows the operator to have direct control over the approval of avoidance paths, which is particularly useful in situations where sensor data may be unreliable or ambiguous.
This mode helps prevent unnecessary or incorrect avoidance maneuvers by requiring human validation before execution.
It is recommended for environments where false positives are likely or when the sensor/perception system's performance is limited.

#### Can this module be customized for specific vehicle types or environments?

The module can be customized by adjusting the rules and parameters that define how it identifies and avoids obstacles.
The avoidance manuever would not be changed by specific vehicle types.

## Appendix: Shift line generation pipeline

### Flow chart of the process

![uml diagram](../../../../../assets/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/image-09-d77fba5016.svg)

### How to decide the path shape

Generate shift points for obstacles with a given lateral jerk. These points are integrated to generate an avoidance path. The detailed process flow for each case corresponding to the obstacle placement are described below. The actual implementation is not separated for each case, but the function corresponding to `multiple obstacle case (both directions)` is always running.

#### One obstacle case

The lateral shift distance to the obstacle is calculated, and then the shift point is generated from the ego vehicle speed and the given lateral jerk as shown in the figure below. A smooth avoidance path is then calculated based on the shift point.

Additionally, the following processes are executed in special cases.

#### Lateral jerk relaxation conditions

- If the ego vehicle is close to the avoidance target, the lateral jerk will be relaxed up to the maximum jerk.
- When returning to the center line after avoidance, if there is not enough distance left to the goal (end of path), the jerk condition will be relaxed as above.

#### Minimum velocity relaxation conditions

There is a problem that we cannot know the actual speed during avoidance in advance. This is especially critical when the ego vehicle speed is 0.
To solve that, this module provides a parameter for the minimum avoidance speed, which is used for the lateral jerk calculation when the vehicle speed is low.

- If the ego vehicle speed is lower than "nominal" minimum speed, use the minimum speed in the calculation of the jerk.
- If the ego vehicle speed is lower than "sharp" minimum speed and a nominal lateral jerk is not enough for avoidance (the case where the ego vehicle is stopped close to the obstacle), use the "sharp" minimum speed in the calculation of the jerk (it should be lower than "nominal" speed).

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/how_to_decide_path_shape_one_object.drawio.svg)

#### Multiple obstacle case (one direction)

Generate shift points for multiple obstacles. All of them are merged to generate new shift points along the reference path. The new points are filtered (e.g. remove small-impact shift points), and the avoidance path is computed for the filtered shift points.

**Merge process of raw shift points**: check the shift length on each path point. If the shift points are overlapped, the maximum shift value is selected for the same direction.

For the details of the shift point filtering, see [filtering for shift points](#filtering-for-shift-points).

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/how_to_decide_path_shape_multi_object_one_direction.drawio.svg)

#### Multiple obstacle case (both direction)

Generate shift points for multiple obstacles. All of them are merged to generate new shift points. If there are areas where the desired shifts conflict in different directions, the sum of the maximum shift amounts of these areas is used as the final shift amount. The rest of the process is the same as in the case of one direction.

![fig](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/images/how_to_decide_path_shape_multi_object_both_direction.drawio.svg)

#### Filtering for shift points

The shift points are modified by a filtering process in order to get the expected shape of the avoidance path. It contains the following filters.

- Quantization: Quantize the avoidance width in order to ignore small shifts.
- Small shifts removal: Shifts with small changes with respect to the previous shift point are unified in the previous shift width.
- Similar gradient removal: Connect two shift points with a straight line, and remove the shift points in between if their shift amount is in the vicinity of the straight line.
- Remove momentary returns: For shift points that reduce the avoidance width (for going back to the center line), if there is enough long distance in the longitudinal direction, remove them.

## Appendix: All parameters

Location of the avoidance specific parameter configuration file: `src/autoware/launcher/planning_launch/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml`.

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| resample\_interval\_for\_planning | float | Path resample interval for avoidance planning path. | 0.3 | N/A |
| resample\_interval\_for\_output | float | Path resample interval for output path. Too short interval increases computational cost for latter modules. | 4.0 | N/A |
| path\_generation\_method | string | Path generation method. | shift\_line\_base | ['shift\_line\_base', 'optimization\_base', 'both'] |
| use\_lane\_type | string | Drivable lane configuration. | opposite\_direction\_lane | ['current\_lane', 'same\_direction\_lane', 'opposite\_direction\_lane'] |
| use\_hatched\_road\_markings | boolean | Extend drivable to hatched road marking area. | true | N/A |
| use\_intersection\_areas | boolean | Extend drivable to intersection area. | true | N/A |
| use\_freespace\_areas | boolean | Extend drivable to freespace area. | true | N/A |
| car.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| car.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| car.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.3 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.2 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| car.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| car.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| car.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| truck.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| truck.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| truck.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.3 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.2 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| truck.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| truck.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| truck.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| bus.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| bus.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| bus.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.3 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.2 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| bus.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| bus.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| bus.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| trailer.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| trailer.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| trailer.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.3 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.2 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| trailer.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| trailer.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| trailer.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| unknown.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| unknown.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| unknown.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | -0.2 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | -0.2 | N/A |
| unknown.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.1 | N/A |
| unknown.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| unknown.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| motorcycle.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| motorcycle.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| motorcycle.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.5 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.5 | N/A |
| motorcycle.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| motorcycle.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| motorcycle.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| bicycle.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| bicycle.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| bicycle.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.3 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.3 | N/A |
| bicycle.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| bicycle.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| bicycle.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| pedestrian.th\_moving\_speed | float | Objects with speed greater than this will be judged as moving ones. | 1.0 | ≥0.0 |
| pedestrian.th\_moving\_time | float | Objects keep moving longer duration than this will be excluded from avoidance target. | 1.0 | N/A |
| pedestrian.longitudinal\_margin | float | Creates an additional longitudinal gap that will prevent the vehicle from getting to near to the obstacle. | 0.0 | N/A |
| lateral\_margin.soft\_margin | float | Lateral distance between ego and avoidance targets. | 0.7 | N/A |
| lateral\_margin.hard\_margin | float | Lateral distance between ego and avoidance targets. | 0.5 | N/A |
| lateral\_margin.hard\_margin\_for\_parked\_vehicle | float | Lateral distance between ego and avoidance targets. | 0.5 | N/A |
| pedestrian.envelope\_buffer\_margin | float | The buffer between raw boundary box of detected objects and enveloped polygon that is used for avoidance path generation. | 0.5 | N/A |
| pedestrian.max\_expand\_ratio | float | This value will be applied envelope\_buffer\_margin according to the distance between the ego and object. | 0.0 | N/A |
| pedestrian.th\_error\_eclipse\_long\_radius | float | This value will be applied to judge whether the eclipse error is to large | 0.6 | N/A |
| target\_object.lower\_distance\_for\_polygon\_expansion | float | If the distance between the ego and object is less than this, the expand ratio will be zero. | 30.0 | N/A |
| target\_object.upper\_distance\_for\_polygon\_expansion | float | If the distance between the ego and object is larger than this, the expand ratio will be max\_expand\_ratio. | 100.0 | N/A |
| target\_type.car | boolean | Enable avoidance maneuver for CAR. | true | N/A |
| target\_type.truck | boolean | Enable avoidance maneuver for TRUCK. | true | N/A |
| target\_type.bus | boolean | Enable avoidance maneuver for BUS. | true | N/A |
| target\_type.trailer | boolean | Enable avoidance maneuver for TRAILER. | true | N/A |
| target\_type.unknown | boolean | Enable avoidance maneuver for UNKNOWN. | true | N/A |
| target\_type.bicycle | boolean | Enable avoidance maneuver for BICYCLE. | true | N/A |
| target\_type.motorcycle | boolean | Enable avoidance maneuver for MOTORCYCLE. | true | N/A |
| target\_type.pedestrian | boolean | Enable avoidance maneuver for PEDESTRIAN. | true | N/A |
| target\_filtering.object\_check\_goal\_distance | float | If the distance between object and goal position is less than this parameter, the module do not return center line. | 20.0 | N/A |
| target\_filtering.object\_check\_return\_pose\_distance | float | If the distance between object and return position is less than this parameter, the module do not return center line. | 20.0 | N/A |
| target\_filtering.max\_compensation\_time | float | For the compensation of the detection lost. The object is registered once it is observed as an avoidance target. When the detection loses, the timer will start and the object will be un-registered when the time exceeds this limit. | 2.0 | N/A |
| target\_filtering.unstable\_classification\_time | float | If the object is classified as UNKNOWN type for less than unstable\_classification\_time seconds, it is probably not an unknown type. | 2.0 | N/A |
| detection\_area.static | boolean | If true, the detection area longitudinal range is calculated based on current ego speed. | false | N/A |
| detection\_area.min\_forward\_distance | float | Minimum forward distance to search the avoidance target. | 50.0 | N/A |
| detection\_area.max\_forward\_distance | float | Maximum forward distance to search the avoidance target. | 150.0 | N/A |
| detection\_area.backward\_distance | float | Backward distance to search the avoidance target. | 10.0 | N/A |
| merging\_vehicle.th\_overhang\_distance | float | Distance threshold to ignore merging/deviating vehicle to/from ego driving lane. The distance represents how the object polygon overlaps ego lane, and it's calculated from polygon overhang point and lane centerline. If the distance is more than this param, the module never avoid the object. (Basically, the ego stops behind of it.) | 0.5 | N/A |
| parked\_vehicle.th\_offset\_from\_centerline | float | Vehicles around the center line within this distance will be excluded from avoidance target. | 1.0 | N/A |
| parked\_vehicle.th\_shiftable\_ratio | float | Vehicles around the center line within this distance will be excluded from avoidance target. | 0.8 | ≥0.0 ≤1.0 |
| parked\_vehicle.min\_road\_shoulder\_width | float | Width considered as a road shoulder if the lane does not have a road shoulder target. | 0.5 | N/A |
| avoidance\_for\_ambiguous\_vehicle.policy | string | Ego behavior policy for ambiguous vehicle. | true | ['auto', 'manual', 'ignore'] |
| avoidance\_for\_ambiguous\_vehicle.closest\_distance\_to\_wait\_and\_see | float | Start avoidance maneuver after the distance to ambiguous vehicle is less than this param. | 10.0 | N/A |
| condition.th\_stopped\_time | float | Never avoid object whose stopped time is less than this param. | 3.0 | N/A |
| condition.th\_moving\_distance | float | Never avoid object which moves more than this param. | 1.0 | N/A |
| traffic\_light.front\_distance | float | If the distance between traffic light and vehicle is less than this parameter, this module will ignore it. | 100.0 | N/A |
| crosswalk.front\_distance | float | If the front distance between crosswalk and vehicle is less than this parameter, this module will ignore it. | 30.0 | N/A |
| crosswalk.behind\_distance | float | If the back distance between crosswalk and vehicle is less than this parameter, this module will ignore it. | 30.0 | N/A |
| wait\_and\_see.target\_behaviors | array | This module doesn't avoid these behaviors vehicle until it gets closer than threshold. | ['MERGING', 'DEVIATING'] | N/A |
| wait\_and\_see.th\_closest\_distance | float | Threshold to check whether the ego gets close enough the ambiguous vehicle. | 10.0 | N/A |
| avoidance\_for\_parking\_violation\_vehicle.policy | string | Ego behavior policy for parking violation vehicle. | ignore | ['auto', 'manual', 'ignore'] |
| condition.th\_road\_border\_distance | float | Distance threshold from road border to determine parking violation. | 0.8 | ≥0.0 |
| avoidance\_for\_adjacent\_lane\_stop\_vehicle.policy | string | Ego behavior policy for adjacent lane stop vehicle. | auto | ['auto', 'manual', 'ignore'] |
| avoidance\_for\_close\_vehicle.policy | string | Ego behavior policy for close vehicle (stopped ego with insufficient avoidance distance). 'auto': avoid automatically in RTC AUTO mode. 'manual': generate candidate path and wait for operator approval. 'ignore': never avoid. | ignore | ['auto', 'manual', 'ignore'] |
| intersection.yaw\_deviation | float | Yaw deviation threshold param to judge if the object is not merging or deviating vehicle. | 0.349 | N/A |
| condition.th\_stopped\_time | float | This module delays avoidance maneuver to see vehicle behavior in freespace. | 5.0 | N/A |
| target\_type.car | boolean | Enable safety\_check for CAR. | true | N/A |
| target\_type.truck | boolean | Enable safety\_check for TRUCK. | true | N/A |
| target\_type.bus | boolean | Enable safety\_check for BUS. | true | N/A |
| target\_type.trailer | boolean | Enable safety\_check for TRAILER. | true | N/A |
| target\_type.unknown | boolean | Enable safety\_check for UNKNOWN. | false | N/A |
| target\_type.bicycle | boolean | Enable safety\_check for BICYCLE. | true | N/A |
| target\_type.motorcycle | boolean | Enable safety\_check for MOTORCYCLE. | true | N/A |
| target\_type.pedestrian | boolean | Enable safety\_check for PEDESTRIAN. | true | N/A |
| safety\_check.enable | boolean | Enable to use safety check feature. | true | N/A |
| safety\_check.check\_current\_lane | boolean | Check objects on current driving lane. | true | N/A |
| safety\_check.check\_shift\_side\_lane | boolean | Check objects on shift side lane. | true | N/A |
| safety\_check.check\_other\_side\_lane | boolean | Check objects on other side lane. | true | N/A |
| safety\_check.check\_unavoidable\_object | boolean | Check collision between ego and unavoidable objects. | true | N/A |
| safety\_check.check\_other\_object | boolean | Check collision between ego and non avoidance target objects. | true | N/A |
| safety\_check.check\_all\_predicted\_path | boolean | Check all prediction path of safety check target objects. | true | N/A |
| safety\_check.safety\_check\_backward\_distance | float | Backward distance to search the dynamic objects. | 100.0 | N/A |
| safety\_check.hysteresis\_factor\_expand\_rate | float | Hysteresis factor that be used for chattering prevention. | 2.0 | N/A |
| safety\_check.hysteresis\_factor\_safe\_count | integer | Hysteresis count that be used for chattering prevention. | 10 | N/A |
| safety\_check.collision\_check\_yaw\_diff\_threshold | float | Max yaw difference between ego and object when doing collision check | 3.1416 | N/A |
| safety\_check.min\_velocity | float | Minimum velocity of the ego vehicle's predicted path. | 1.38 | N/A |
| safety\_check.max\_velocity | float | Maximum velocity of the ego vehicle's predicted path. | 50.0 | N/A |
| safety\_check.time\_resolution | float | Time resolution for the ego vehicle's predicted path. | 0.5 | N/A |
| safety\_check.time\_horizon\_for\_front\_object | float | Time horizon for predicting front objects. | 3.0 | N/A |
| safety\_check.time\_horizon\_for\_rear\_object | float | Time horizon for predicting rear objects. | 10.0 | N/A |
| safety\_check.delay\_until\_departure | float | Delay until the ego vehicle departs. | 1.0 | N/A |
| safety\_check.extended\_polygon\_policy | string | See https://github.com/autowarefoundation/autoware\_universe/pull/6336. | along\_path | ['rectangle', 'along\_path'] |
| safety\_check.expected\_front\_deceleration | float | The front object's maximum deceleration when the front vehicle perform sudden braking. | -1.0 | N/A |
| safety\_check.expected\_rear\_deceleration | float | The rear object's maximum deceleration when the rear vehicle perform sudden braking. | -1.0 | N/A |
| safety\_check.rear\_vehicle\_reaction\_time | float | The reaction time of the rear vehicle driver which starts from the driver noticing the sudden braking of the front vehicle until the driver step on the brake. | 2.0 | N/A |
| safety\_check.rear\_vehicle\_safety\_time\_margin | float | The time buffer for the rear vehicle to come into complete stop when its driver perform sudden braking. | 1.0 | N/A |
| safety\_check.lateral\_distance\_max\_threshold | float | The lateral distance threshold that is used to determine whether lateral distance between two object is enough and whether lane change is safe. | 2.0 | N/A |
| safety\_check.longitudinal\_distance\_min\_threshold | float | The longitudinal distance threshold that is used to determine whether longitudinal distance between two object is enough and whether lane change is safe. | 3.0 | N/A |
| safety\_check.longitudinal\_velocity\_delta\_time | float | The time multiplier that is used to compute the actual gap between vehicle at each predicted points (not RSS distance) | 0.0 | N/A |
| lateral.th\_avoid\_execution | float | The lateral distance deviation threshold between the current path and suggested avoidance point to execute avoidance. | 0.09 | N/A |
| lateral.th\_small\_shift\_length | float | The shift lines whose lateral offset is less than this will be applied with other ones. | 0.101 | N/A |
| lateral.soft\_drivable\_bound\_margin | float | Keep distance to drivable bound. | 0.3 | N/A |
| lateral.hard\_drivable\_bound\_margin | float | Keep distance to drivable bound. | 0.3 | N/A |
| lateral.max\_right\_shift\_length | float | Maximum shift length for right direction | 5.0 | N/A |
| lateral.max\_left\_shift\_length | float | Maximum shift length for left direction. | 5.0 | N/A |
| lateral.max\_deviation\_from\_lane | float | Use in validation phase to check if the avoidance path is in drivable area. | 0.2 | N/A |
| lateral.ratio\_for\_return\_shift\_approval | float | This parameter is added to allow waiting for the return of shift approval until the occlusion behind the avoid target is clear. | 0.5 | ≥0.0 ≤1.0 |
| longitudinal.min\_prepare\_time | float | Avoidance shift starts from point ahead of this time x ego speed at least. | 1.0 | N/A |
| longitudinal.max\_prepare\_time | float | Avoidance shift starts from point ahead of this time x ego speed if possible. | 2.0 | N/A |
| longitudinal.min\_prepare\_distance | float | Minimum prepare distance. | 1.0 | N/A |
| longitudinal.min\_slow\_down\_speed | float | Minimum slow speed for avoidance prepare section. | 1.38 | N/A |
| longitudinal.buf\_slow\_down\_speed | float | Buffer for controller tracking error. Basically, vehicle always cannot follow velocity profile precisely. Therefore, the module inserts lower speed than target speed that satisfies conditions to avoid object within accel/jerk constraints so that the avoidance path always can be output even if the current speed is a little bit higher than target speed. | 0.56 | N/A |
| longitudinal.nominal\_avoidance\_speed | float | Nominal avoidance speed. | 8.33 | N/A |
| longitudinal.consider\_front\_overhang | boolean | Flag to consider vehicle front overhang in shift line generation logic. | True | N/A |
| longitudinal.consider\_rear\_overhang | boolean | Flag to consider vehicle rear overhang in shift line generation logic. | True | N/A |
| goal.enable | boolean | Insert stop point in order to return original lane before reaching goal. | true | N/A |
| goal.buffer | float | Buffer distance to return original lane before reaching goal. | 3.0 | N/A |
| traffic\_light.enable | boolean | Insert stop point in order to return original lane before reaching traffic light. | true | N/A |
| traffic\_light.buffer | float | Buffer distance to return original lane before reaching traffic light. | 3.0 | N/A |
| stop.max\_distance | float | Maximum stop distance in the situation where avoidance maneuver is not approved or in yield maneuver. | 20.0 | N/A |
| stop.stop\_buffer | float | Buffer distance in the situation where avoidance maneuver is not approved or in yield maneuver. | 1.0 | N/A |
| yield.enable | boolean | Flag to enable yield maneuver. | true | N/A |
| yield.enable\_during\_shifting | boolean | Flag to enable yield maneuver during shifting. | false | N/A |
| cancel.enable | boolean | Flag to enable cancel maneuver. | true | N/A |
| force.duration\_time | float | force deactivate duration time | 2.0 | N/A |
| policy.detection\_reliability | string | policy for module behavior. select 'reliable' or 'not\_enough'. 'reliable': this module fully trusts the perception results and makes all decisions automatically. 'not\_enough': because the perception results are not fully reliable, the system waits for the operator's judgment in situations where long-range perception is required. | reliable | ['reliable', 'not\_enough'] |
| policy.make\_approval\_request | string | policy for rtc request. select `per_shift_line` or `per_avoidance_maneuver`. `per_shift_line`: request approval for each shift line. `per_avoidance_maneuver`: request approval for avoidance maneuver (avoid + return). | per\_shift\_line | ['per\_shift\_line', 'per\_avoidance\_maneuver'] |
| policy.deceleration | string | policy for vehicle slow down behavior. select `best_effort` or `reliable`. `best_effort`: slow down deceleration & jerk are limited by constraints but there is a possibility that the vehicle can't stop in front of the vehicle. `reliable`: insert stop or slow down point with ignoring decel/jerk constraints. make it possible to increase chance to avoid but uncomfortable deceleration maybe happen. | best\_effort | ['reliable', 'best\_effort'] |
| policy.lateral\_margin | string | policy for voidance lateral margin. select `best_effort` or `reliable`. `best_effort`: output avoidance path with shorten lateral margin when there is no enough longitudinal margin to avoid. `reliable`: module output avoidance path with safe (rtc cooperate) state only when the vehicle can avoid with expected lateral margin. | best\_effort | ['reliable', 'best\_effort'] |
| policy.use\_shorten\_margin\_immediately | boolean | if true, module doesn't wait deceleration and outputs avoidance path by best effort margin. | true | N/A |
| policy.candidate\_path\_turn\_signal | string | policy for turn signal output during planWaitingApproval (candidate path). 'none': never output turn signal for candidate paths. 'stopped\_candidate': output turn signal only when the vehicle is stopped and a candidate path exists (default behavior). 'all\_candidate': always output turn signal whenever a candidate path exists. 'stop\_on\_approval': no turn signal during candidate phase; after approval, hold a stop point and turn signal for turn\_signal\_on\_approval\_hold\_duration seconds before proceeding with avoidance. | none | ['none', 'stopped\_candidate', 'all\_candidate', 'stop\_on\_approval'] |
| policy.turn\_signal\_on\_approval\_hold\_duration | float | duration [s] to hold stop and turn signal after approval (used by 'stop\_on\_approval' policy) | 3.0 | ≥0.0 |
| lateral.velocity | array | Velocity array to decide current lateral accel/jerk limit. | [1.0, 1.38, 11.1] | N/A |
| lateral.max\_accel\_values | array | Avoidance path gets sharp up to this accel limit when there is not enough distance from ego. | [0.5, 0.5, 0.5] | N/A |
| min\_jerk\_values.avoid | array | Minimum lateral jerk constraint when laterally shifting to avoid an obstacle. | [0.2, 0.2, 0.2] | N/A |
| min\_jerk\_values.return | array | Minimum lateral jerk constraint when laterally shifting to return to the original lane. | [0.2, 0.2, 0.2] | N/A |
| lateral.max\_jerk\_values | array | Avoidance path gets sharp up to this jerk limit when there is not enough distance from ego. | [1.0, 1.0, 1.0] | N/A |
| longitudinal.nominal\_deceleration | float | Nominal deceleration limit. | -1.0 | N/A |
| longitudinal.nominal\_jerk | float | Nominal jerk limit. | 0.5 | N/A |
| longitudinal.max\_deceleration | float | Max deceleration limit. | -1.5 | N/A |
| longitudinal.max\_jerk | float | Max jerk limit. | 1.0 | N/A |
| longitudinal.max\_acceleration | float | Maximum acceleration during avoidance. | 0.5 | N/A |
| longitudinal.min\_velocity\_to\_limit\_max\_acceleration | float | If the ego speed is faster than this param, the module applies acceleration limit `max_acceleration`. | 2.78 | N/A |
| trim.quantize\_size | float | Lateral shift length quantize size. | 0.1 | N/A |
| trim.th\_similar\_grad\_1 | float | Lateral shift length threshold to merge similar gradient shift lines. | 0.1 | N/A |
| trim.th\_similar\_grad\_2 | float | Lateral shift length threshold to merge similar gradient shift lines. | 0.2 | N/A |
| trim.th\_similar\_grad\_3 | float | Lateral shift length threshold to merge similar gradient shift lines. | 0.5 | N/A |
| debug.enable\_other\_objects\_marker | boolean | Publish other objects marker. | false | N/A |
| debug.enable\_other\_objects\_info | boolean | Publish other objects detail information. | false | N/A |
| debug.enable\_detection\_area\_marker | boolean | Publish detection area. | false | N/A |
| debug.enable\_drivable\_bound\_marker | boolean | Publish drivable area boundary. | false | N/A |
| debug.enable\_safety\_check\_marker | boolean | Publish safety check information. | false | N/A |
| debug.enable\_shift\_line\_marker | boolean | Publish shift line information. | false | N/A |
| debug.enable\_lane\_marker | boolean | Publish lane information. | false | N/A |
| debug.enable\_misc\_marker | boolean | Publish misc markers. | false | N/A |
