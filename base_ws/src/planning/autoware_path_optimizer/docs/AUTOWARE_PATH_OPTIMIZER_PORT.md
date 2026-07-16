# Autoware Path Optimizer ROS 1 MVP Port

## Pipeline position

```text
mission_planner
  -> behavior_path_planner
  -> behavior_velocity_planner
  -> autoware_path_smoother
  -> autoware_path_optimizer
  -> motion_velocity_planner
```

The package is kept at the same planning directory level and uses the original
Autoware package name. No new planning message was required.

## Topic contract

| Direction | ROS 1 topic | Message |
| --- | --- | --- |
| Input | `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path` | `autoware_planning_msgs/Path` |
| Input | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| Output | `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | `autoware_planning_msgs/Trajectory` |

These names were checked directly against the ROS 2 Planning Simulator runtime.

## MVP behavior

The optimizer crops the input around the ego, resamples the optimization horizon
at 1.0 m, and optimizes lateral offsets in the path Frenet frame. The objective
penalizes lateral tracking error, heading change, curvature, and curvature rate.
The first points and a contained goal are fixed to stabilize the trajectory.

Left and right path bounds are converted to vehicle-center lateral limits using
the configured vehicle width and road clearance. A result outside those limits
is rejected. If optimization fails or the corridor is narrower than the vehicle,
the node reuses its previous output or the input path and inserts a stop before
the first unsafe point. Input velocity and the first exact stop position are
preserved. Final output spacing is 0.5 m.

## Difference from the full Autoware implementation

This is an MVP, not the complete Autoware MPT implementation.

- It uses a fixed-normal Frenet optimization solved by SciPy L-BFGS-B instead of
  the full linearized bicycle-state QP with OSQP or acados.
- It constrains the vehicle center using vehicle half-width rather than all
  upstream three-circle footprint constraint variants.
- Obstacles affect this optimizer only when an earlier planner has removed them
  from the drivable bounds. There is no direct predicted-object input.
- Previous-trajectory reuse, stop fallback, velocity inheritance, goal fixing,
  steering-limit validation, and drivable-bound validation are implemented.

## Validation

References were captured from the Path Optimizer running in the ROS 2 Autoware
Planning Simulator with the exact same Path Smoother output and odometry replayed
into the ROS 1 node.

### Scenario 1: stop line 103624

1. The ego vehicle follows a mostly straight K-City v44 route.
2. The upstream Behavior Velocity Planner inserts a zero-velocity stop at stop
   line `103624`.
3. The Path Smoother output, including its drivable bounds and velocity profile,
   is sent to both the ROS 2 reference optimizer and this ROS 1 port.
4. Validation checks that lateral optimization does not move the stop position,
   exceed the road bounds, or change the upstream velocity profile.

```bash
roslaunch autoware_path_optimizer path_optimizer_stop_line_validation.launch
```

| Metric | Result |
| --- | ---: |
| Geometry RMSE | 0.0059 m |
| Maximum geometry error | 0.0339 m |
| Length error | 0.0007 m |
| Stop-position error | 0.0000 m |
| Velocity RMSE | 0.0000 m/s |
| Maximum inferred steering | 0.0882 rad |

### Scenario 2: crosswalk 201266 curved route

1. The ego vehicle follows a curved K-City v44 route through crosswalk lanelet
   `201266`.
2. The curved geometry exercises heading, curvature, steering, endpoint, and
   vehicle-width constraints that are nearly inactive in Scenario 1.
3. The same Path Smoother input and odometry are replayed into the ROS 2 and ROS
   1 optimizers.
4. Validation compares both trajectories and confirms that the ROS 1 result
   remains inside the left and right drivable bounds.
5. This is the stronger of the captured K-City scenarios: the ROS 2 optimizer
   moves the Path Smoother input by up to 0.159 m near `(2556.96, 23881.90)`.
   The validation requires at least 0.10 m of optimizer effect, so a pass cannot
   come from a nearly unchanged path.

```bash
roslaunch autoware_path_optimizer path_optimizer_crosswalk_validation.launch
```

| Metric | Result |
| --- | ---: |
| Geometry RMSE | 0.0700 m |
| Maximum geometry error | 0.1898 m |
| Length error | 0.3679 m |
| Stop-position error | 0.0023 m |
| Velocity RMSE | 0.0000 m/s |
| Maximum inferred steering | 0.3952 rad |
| ROS 2 maximum change from Smoother input | 0.1587 m |
| ROS 1 maximum change from Smoother input | 0.3433 m |
| Strong-region ROS 1/ROS 2 RMSE | 0.1411 m |
| Strong-region maximum error | 0.1898 m |

All geometry, endpoint, length, velocity, stop, steering-limit, and drivable-area
checks passed. In RViz, large gray points are the Path Smoother input, medium
cyan points are the ROS 2 reference, and small magenta points are the ROS 1
output. All paths use their actual coordinates without a display offset. The
orange and yellow highlighted points show the input and output at the strongest
ROS 2 optimization position. The bright pink point is the strongest ROS 1
optimization position. Green/orange lines are the left/right bounds, and the
cyan/magenta spheres are the first zero-velocity positions. The ROS 1 MVP moves
the path more aggressively than ROS 2 at the peak, but its strong-region RMSE
and maximum error remain inside the configured 0.15 m and 0.25 m tolerances.

## Integrated launch

```bash
roslaunch bringup mission_bpp_bvp_path_smoother_path_optimizer_pipeline.launch \
  map_file:=$HOME/AJOUNICE2026/lanelet2_map_v44.osm \
  map_projector_type:=MGRS
```

The complete pipeline was also run with the K-City v44 stop-line reference
input. Runtime message counts were:

| Stage | Points | Final velocity | Zero-velocity points |
| --- | ---: | ---: | ---: |
| Behavior Velocity Planner path | 2104 | 0.0 m/s | 339 |
| Path Smoother path | 415 | 0.0 m/s | 70 |
| Path Optimizer trajectory | 415 | 0.0 m/s | 70 |

This confirms that the launch wiring is live and that the Path Optimizer keeps
the smoothed path's stop profile through the final trajectory output.
