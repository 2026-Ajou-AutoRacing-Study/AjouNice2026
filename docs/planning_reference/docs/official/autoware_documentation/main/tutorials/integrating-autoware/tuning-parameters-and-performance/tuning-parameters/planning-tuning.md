---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Tuning planning
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/
rendered_html: rendered/autoware_documentation_html/index-416.html
generated: 2026-06-24T15:00:00.022041+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Tuning planning

In this section, we will update, evaluate, and fine-tune the Autoware planning modules,
with a specific emphasis on the lane driving modules within the YTU campus environment.
The Autoware planning side consists of the following main lane driving sections:
**Behavior planning and motion planning**. Our focus will be on fine-tuning these modules to
enhance our planning performance in the campus environment.

## Introduction

## Planning parameter tuning

### Behavior planning tuning

#### Behavior velocity planner

The Behavior velocity planner is a planner that adjusts velocity based on traffic rules.
It loads modules as plugins. Please refer to the package documentation for more
information about these modules.

To enable or disable Behavior velocity planner modules,
we will enable or disable the necessary plugin modules in the [`default_preset.yaml` file](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml) according to our preferences.

- For example, in the YTU campus environment, there are many speed bumps, pedestrians, and specific areas where autonomous driving is not permitted.
  We will enable the following three modules to handle these conditions:
  - Speed bump module
  - Out of lane module

To enable these modules,
we need to set the launch\_ argument
to `true` for these modules in the `default_preset.yaml` parameter file:

[`default_preset.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml) parameter file:

```
arg:
  name: launch_speed_bump_module
-   default: "false"
+   default: "true"
arg:
  name: launch_out_of_lane_module
-   default: "false"
+   default: "true"
```

#### Speed bump module tuning

- Our vehicle's cruising speed is set to `15 km/h`.
  Therefore,
  we will decrease the default speed bump velocity limits in the `speed_bump.param.yaml`
  to allow for minimum and maximum speeds when going over speed bumps.

[`speed_bump.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/speed_bump.param.yaml) parameter file:

```
- min_speed: 1.39  # [m/s] = [5 kph]
+ min_speed: 1.11  # [m/s] = [4 kph]
- max_speed: 2.78  # [m/s] = [10 kph]
+ max_speed: 2.22  # [m/s] = [8 kph]
```

- Also, we will increase the slow\_start\_margin parameter to provide
  a greater margin for slowing down when approaching a speed bump.
  Please refer to the speed bump module page for more information
  about these parameters. It is essential to fine-tune these modules
  based on your specific vehicle and environmental conditions.

[`speed_bump.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/speed_bump.param.yaml) parameter file:

```
- slow_start_margin: 1.0 [m]
+ slow_start_margin: 2.0 [m]
```

- The following image illustrates the virtual wall created by the slow start margin
  of the speed bump module. If you increase or decrease the `slow_start_margin` parameter,
  you will observe that the position of the virtual wall changes is relative to the speed bump.

![speed-bump-tuning](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/images/speed-bump.png)

The speed bump module slow start margin is demonstrated as a virtual wall in RViz.

#### Avoidance

The Avoidance module plays a pivotal role in Autoware's behavior planning scene modules,
offering rule-based avoidance capabilities. It provides the flexibility to define behavior
based on intuitive parameters like lateral jerk and avoidance distance margin, enabling
adaptable avoidance strategies tailored to your specific environment. This module operates
within lane constraints, requiring access to lane structure information to ensure compliance
with traffic rules. For example, it triggers indicator signals when the vehicle crosses a lane.
The key distinction between motion and behavior modules in the planning stack is their consideration
of traffic rules.
Please refer to the [`Avoidance Module` page](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/) for more information about the module's capabilities.

We will modify and update certain avoidance rules and margin parameters
to handle with the specific conditions of our YTU campus environment
and the capabilities of our vehicle.

- First, we will disable the use of opposite lanelets while avoiding,
  as we do not want to utilize them in our YTU Campus environment.
  The default avoidance behavior in our environment is as depicted in the image below.

![avoidance-opposite-lane](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/images/avoidance-opposite-lane.png)

When `use\_lane\_type` is set to `opposite\_direction\_lane`,
the path can be shifted to the opposite lane if the ego vehicle cannot fit in the current lane.

- To disable the use of the opposite lanelet in the avoidance, we will set `use_lane_type`
  to `same_direction_lane` in the `static_obstacle_avoidance.param.yaml` file:

[`static_obstacle_avoidance.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml) parameter file:

```
  avoidance:
-   use_lane_type: "opposite_direction_lane"
+   use_lane_type: "same_direction_lane"
```

Now, we expect the avoidance module not to utilize the opposite lane in our area.
If the vehicle cannot fit past an obstacle in the same lanelet, it will come to a stop,
as depicted in the image below:

![avoidance-current-lane](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/images/avoidance-current-lane.png)

When `use\_lane\_type` is set to `same\_direction\_lane`,
the path cannot be shifted to the opposite lane, even if the ego vehicle cannot fit in the current lane.

Since we disabled the use of opposite lanelets for the Avoidance module,
there may be instances where we cannot avoid objects due to the shifted path
not fitting within the available lanelet.
As a small vehicle, we will reduce the `soft_margin` parameter to decrease the distance
between objects and the ego vehicle.
You can adjust these changes to any target object according to your preference.
For more information on these parameters, please refer to
the [avoidance module documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/).

[`static_obstacle_avoidance.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml) parameter file:

```
  avoidance:
    target_object:
      car:
        lateral_margin:
-         soft_margin: 0.5                            # [m]
+         soft_margin: 0.3                            # [m]
```

![avoidance-current-lane](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/images/avoidance-lateral-margin.png)

The Avoidance module's behavior changes when the `soft\_margin` value is decreased,
such that the shifted path fits within the lanelet.

- Also, you can choose which objects can be avoided along the path.
  In the YTU campus environment, as mentioned earlier, there are many
  pedestrians, and we do not want to perform avoidance maneuvers for them.
  Therefore, we will disable the avoidance maneuver for pedestrians by
  modifying the target value in the [`static_obstacle_avoidance.param.yaml` file](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml).
  You can disable any target object according to your preference.

[`static_obstacle_avoidance.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml) parameter file:

```
  avoidance:
    target_filtering:
      target_type:
-       pedestrian: true
+       pedestrian: false
```

### Motion Planning Tuning

#### Path optimizer

The path optimizer generates a kinematically feasible
and collision-free trajectory based on the input path and drivable area.
It updates the trajectory's position and orientation but retains the velocity from the input path.
Please refer to the [Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)
and [Model Predictive Trajectory (MPT)](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/mpt/)
documentation page for more information about this package.

- The YTU Campus environment features numerous U-turns, narrow roads, and roundabouts.
  Given our vehicle's maximum steering angle is insufficient for these conditions,
  we must consider our steering angle limit (defined in the vehicle\_info.param.yaml)
  to navigate these road types without exceeding road boundaries.
  Therefore, we will enable the steer\_limit\_constraint parameter for the path optimizer:

[`path_optimizer.param.yaml`](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_optimizer/config/path_optimizer.param.yaml) parameter file:

```
  mpt:
    option:
-     steer_limit_constraint: false
+     steer_limit_constraint: true
```

- Additionally, we will modify our mechanism for checking outside the drivable area.
  By default, the path optimizer checks the four corner points of the MPT footprint.
  However, this may lead to incorrect information in some situations, as shown in the following image.

![obstacle-avoidance-check](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/images/obstacle-avoidance-planner.svg)

The path optimizer's default method for checking the drivable area is
to check the four corner points of the MPT footprint.
This may not be sufficient to detect if the footprint extends beyond the lane boundaries.

- To address this issue,
  we will enable the `use_footprint_polygon_for_outside_drivable_area_check` parameter
  to consider the footprint as a polygon and check if it exceeds the Lanelet2 boundaries.

[`path_optimizer.param.yaml`](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_optimizer/config/path_optimizer.param.yaml) parameter file:

```
  option:
-   use_footprint_polygon_for_outside_drivable_area_check: false # If false, only the footprint's corner points are considered.
+   use_footprint_polygon_for_outside_drivable_area_check: true # If false, only the footprint's corner points are considered.
```

#### Obstacle stop planner

Autoware implements two motion stop planners:
the obstacle stop planner and the obstacle cruise planner.
We use the obstacle stop planner in the campus environment because it considers the input point cloud to insert stop points into the trajectory
(more safety).
The obstacle cruise planner, on the other hand, uses dynamic objects instead of the point cloud.
This provides higher accuracy than estimating the velocity on the planning side.

To determine which stop planner is being used,
please update the motion\_stop\_planner\_type parameter in the `default_preset.yaml` file:

[`default_preset.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml) parameter file:

```
  - arg:
      name: motion_stop_planner_type
      default: obstacle_stop_planner
      # option: obstacle_stop_planner
      #         obstacle_cruise_planner
      #         none
```

In our YTU environment and test vehicle conditions,
due to the short vehicle width, the trajectory is also short.
To improve the detection range of obstacle stop planner,
we will increase the lateral margin for detection area.
For more information on the parameters and inner working algorithms of the obstacle stop planner,
please refer to the [documentation page](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/).

[`obstacle_stop_planner.param.yaml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml) parameter file:

```
  detection_area:
-   lateral_margin: 0.0                  # margin [m]
+   lateral_margin: 0.6                  # margin [m]
```

The following image illustrates the motion behavior of the trajectory,
considering the lateral\_margin and max\_longitudinal\_margin parameters.

![obstacle-stop-planner](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/tuning-parameters-and-performance/tuning-parameters/planning-tuning/images/obstacle-stop-planner.png)

The obstacle stop planner checks the detection area with the given lateral margin.
If any points from the input cloud are detected,
it inserts a stop point behind the detected point cloud with the maximum longitudinal margin.

Autoware has a lot of changeable parameters and methods.
The mentioned sample parameter tuning examples are just a small part of what is possible.
For more information on the modules and their parameters,
please refer to the [Autoware Universe documentation](https://autowarefoundation.github.io/autoware.universe/main/).
