---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Planning Launch Files
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/launch-autoware/planning/
rendered_html: rendered/autoware_documentation_html/index-407.html
generated: 2026-06-24T14:59:59.706371+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning Launch Files

## Overview

The Autoware planning stacks start
launching at `autoware_launch.xml` as mentioned on the [Launch Autoware](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/launch-autoware/) page.
The `autoware_launch` package includes `tier4_planning_component.launch.xml`
for initiating planning launch files invocation from `autoware_launch.xml`.
The diagram below illustrates the flow of Autoware planning launch files within the autoware\_launch and autoware\_universe packages.

![planning-launch-flow](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/launch-autoware/planning/images/planning_launch_flow.svg)

Autoware planning launch flow diagram

Note

The Autoware project is a large project.
Therefore, as we manage the Autoware project, we utilize specific
arguments in the launch files.
ROS 2 offers an argument-overriding feature for these launch files.
Please refer to [the official ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameter-overrides) for further information.
For instance,
if we define an argument at the top-level launch,
it will override the value on lower-level launches.

## tier4\_planning\_component.launch.xml

The `tier4_planning_component.launch.xml` launch file is the main planning component launch at the `autoware_launch` package.
This launch file calls `planning.launch.xml` at [tier4\_planning\_launch](https://github.com/autowarefoundation/autoware_launch/tree/main/tier4_universe_launch/tier4_planning_launch) package from `autoware_universe` repository.
We can modify planning launch arguments at tier4\_planning\_component.launch.xml.
Also,
we can add any other necessary arguments
that we want
to change it since `tier4_planning_component.launch.xml` is the top-level launch file of other planning launch files.
Here are some predefined planning launch arguments:

- **`use_experimental_lane_change_function:`** This argument enables
  `enable_collision_check_at_prepare_phase`, `use_predicted_path_outside_lanelet`,
  and `use_all_predicted_path` options for Autoware for experimental lane changing
  (for more information, please refer to [lane\_change documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/)).
  The default value is True.
  To set it to False, make the following change in the `tier4_planning_component.launch.xml` file:

  ```
  - <arg name="use_experimental_lane_change_function" default="true"/>
  + <arg name="use_experimental_lane_change_function" default="false"/>
  ```

- **`cruise_planner_type:`** There are two types of cruise planners in Autoware: [obstacle\_stop\_planner](https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/)
  and [obstacle\_cruise\_planner](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/). For specifications on these cruise planner types,
  please refer to the package documentation. The default cruise planner is `obstacle_stop_planner`.
  To change it to obstacle\_cruise\_planner, update the argument value in the `tier4_planning_component.launch.xml` file:

  ```
  - <arg name="cruise_planner_type" default="obstacle_stop_planner" description="options: obstacle_stop_planner, obstacle_cruise_planner, none"/>
  + <arg name="cruise_planner_type" default="obstacle_cruise_planner" description="options: obstacle_stop_planner, obstacle_cruise_planner, none"/>
  ```

- **`use_surround_obstacle_check:`** This argument enables the [surround\_obstacle\_checker](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_surround_obstacle_checker/)
  for Autoware. If you want to disable it, you can do in the
  `tier4_planning_component.launch.xml` file:

  ```
  - <arg name="use_surround_obstacle_check" default="true"/>
  + <arg name="use_surround_obstacle_check" default="false"/>
  ```

- **`velocity_smoother_type:`** This argument specifies the type of smoother
  for the [motion\_velocity\_smoother](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_velocity_smoother/) package. Please consult the documentation
  for detailed information about available smoother types. For instance, if
  you wish to change your smoother type from JerkFiltered to L2, you can do
  in the tier4\_planning\_component.launch.xml file.:

  ```
  - <arg name="velocity_smoother_type" default="JerkFiltered" description="options: JerkFiltered, L2, Analytical, Linf(Unstable)"/>
  + <arg name="velocity_smoother_type" default="L2" description="options: JerkFiltered, L2, Analytical, Linf(Unstable)"/>
  ```

Note

You can also use this arguments as command line arguments:

```
ros2 launch autoware_launch autoware.launch.xml ... use_surround_obstacle_check:=false velocity_smoother_type:=L2 ...
```

The predefined arguments in tier4\_planning\_component.launch.xml have been explained above.
However, numerous planning arguments are included in the autoware\_launch planning config parameters.
