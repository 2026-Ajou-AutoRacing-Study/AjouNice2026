---
source_type: rendered_autoware_docs
site: autoware_universe
title: Debug
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/debug/
rendered_html: rendered/autoware_universe_html/index-163.html
generated: 2026-06-24T14:59:32.111918+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Debug

## Debug visualization

The visualization markers of the planning flow (Input, Model Predictive Trajectory, and Output) are explained here.

All the following markers can be visualized by

```
ros2 launch autoware_path_optimizer launch_visualiation.launch.xml vehilce_model:=sample_vehicle
```

The `vehicle_model` must be specified to make footprints with vehicle's size.

### Input

- **Path**
  - The path generated in the `behavior` planner.
  - The semitransparent and thick, green and red band, that is visualized by default.

![path](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/path_visualization.png)

- **Path Footprint**
  - The path generated in the `behavior` planner is converted to footprints.
  - NOTE:
    - Check if there is no high curvature.
    - The path may be outside the drivable area in some cases, but it is okay to ignore it since the `behavior` planner does not support it.

![path_footprint](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/path_footprint_visualization.png)

- **Drivable Area**
  - The Drivable area generated in the `behavior` planner.
  - The skyblue left and right line strings, that is visualized by default.
  - NOTE:
    - Check if the path is almost inside the drivable area.
      - Then, the `path_optimizer` will try to make the trajectory fully inside the drivable area.
    - During avoidance or lane change by the `behavior` planner, please make sure that the drivable area is expanded correctly.

![drivable_area](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/drivable_area_visualization.png)

### Model Predictive Trajectory (MPT)

- **MPT Reference Trajectory**
  - The reference trajectory points of model predictive trajectory.

![mpt_ref_traj](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/mpt_ref_traj_visualization.png)

- **MPT Fixed Trajectory**
  - The fixed trajectory points as a constraint of model predictive trajectory.

![mpt_fixed_traj](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/mpt_fixed_traj_visualization.png)

- **Boundaries' Width**
  - The boundaries' width is calculated from the drivable area line strings.

![bounds](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/bounds_visualization.png)

- **Vehicle Circles**
  - The vehicle's shape is represented by a set of circles.
  - The `path_optimizer` will try to make the these circles inside the above boundaries' width.

![vehicle_circles](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/vehicle_circles_visualization.png)

- **Vehicle Circles on Trajectory**
  - The vehicle's circles on the MPT trajectory.
  - Check if the circles are not so big compared to the road's width.

![vehicle_traj_circles](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/vehicle_traj_circles_visualization.png)

- **MPT Trajectory**
  - The optimized trajectory points by model predictive trajectory.
  - The footprints are supposed to be fully inside the drivable area.

![mpt_traj](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/mpt_traj_visualization.png)

### Output

- **Trajectory**
  - The output trajectory.
  - The dark and thin, green and red band, that is visualized by default.

![traj](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/traj_visualization.png)

- **Trajectory Footprint**
  - The output trajectory is converted to footprints.

![traj_footprint](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/traj_footprint_visualization.png)

## Calculation time

The `path_optimizer` consists of many functions such as boundaries' width calculation, collision-free planning, etc.
We can see the calculation time for each function as follows.

### Raw data

Enable `option.enable_calculation_time_info` or echo the topic as follows.

```sh
$ ros2 topic echo /planning/scenario\_planning/lane\_driving/motion\_planning/path\_optimizer/debug/calculation\_time --field data

---

```
    insertFixedPoint:= 0.008 [ms]
    getPaddedTrajectoryPoints:= 0.002 [ms]
    updateConstraint:= 0.741 [ms]
    optimizeTrajectory:= 0.101 [ms]
    convertOptimizedPointsToTrajectory:= 0.014 [ms]
  getEBTrajectory:= 0.991 [ms]
      resampleReferencePoints:= 0.058 [ms]
      updateFixedPoint:= 0.237 [ms]
      updateBounds:= 0.22 [ms]
      updateVehicleBounds:= 0.509 [ms]
    calcReferencePoints:= 1.649 [ms]
    calcMatrix:= 0.209 [ms]
    calcValueMatrix:= 0.015 [ms]
      calcObjectiveMatrix:= 0.305 [ms]
      calcConstraintMatrix:= 0.641 [ms]
      initOsqp:= 6.896 [ms]
      solveOsqp:= 2.796 [ms]
    calcOptimizedSteerAngles:= 9.856 [ms]
    calcMPTPoints:= 0.04 [ms]
  getModelPredictiveTrajectory:= 12.782 [ms]
optimizeTrajectory:= 12.981 [ms]
applyInputVelocity:= 0.577 [ms]
insertZeroVelocityOutsideDrivableArea:= 0.81 [ms]
  getDebugMarker:= 0.684 [ms]
  publishDebugMarker:= 4.354 [ms]
publishDebugMarkerOfOptimization:= 5.047 [ms]
```

generateOptimizedTrajectory:= 20.374 [ms]
extendTrajectory:= 0.326 [ms]
publishDebugData:= 0.008 [ms]
onPath:= 20.737 [ms]
```

### Plot

With the following script, any calculation time of the above functions can be plot.

```
ros2 run autoware_path_optimizer calculation_time_plotter.py
```

![calculation_time_plot](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/debug/calculation_time_plot.png)

You can specify functions to plot with the `-f` option.

```
ros2 run autoware_path_optimizer calculation_time_plotter.py -f "onPath, generateOptimizedTrajectory, calcReferencePoints"
```

## Q&A for Debug

### The output frequency is low

Check the function which is comparatively heavy according to [this information](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/debug/#calculation-time).

For your information, the following functions for optimization and its initialization may be heavy in some complicated cases.

- MPT
  - `initOsqp`
  - `solveOsqp`

### When a part of the trajectory has high curvature

Some of the following may have an issue.
Please check if there is something weird by the visualization.

- Input Path
- Drivable Area
- Boundaries' Width

### When the trajectory's shape is zigzag

Some of the following may have an issue.
Please check if there is something weird by the visualization.

- Vehicle Circles on Trajectory
