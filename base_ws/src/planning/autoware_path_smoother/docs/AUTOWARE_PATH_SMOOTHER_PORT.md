# Autoware Path Smoother ROS 1 Port

## Position in the planning pipeline

This is a standalone ROS 1 package at the same directory level as
`behavior_velocity_planner`. It is not a BVP submodule.

```text
mission_planner
  -> behavior_path_planner
  -> behavior_velocity_planner
  -> autoware_path_smoother
  -> path_optimizer (next porting step)
```

The package and internal source roles follow Autoware Universe 0.51.0:

```text
autoware_path_smoother/
  config/elastic_band_smoother.param.yaml
  launch/elastic_band_smoother.launch
  scripts/elastic_band_smoother.py
  src/autoware_path_smoother/elastic_band.py
  src/autoware_path_smoother/replan_checker.py
  src/autoware_path_smoother/trajectory_utils.py
  validation/
```

## ROS topic contract

| Direction | ROS 1 topic | Message |
| --- | --- | --- |
| Input | `/planning/scenario_planning/lane_driving/behavior_planning/path` | `autoware_planning_msgs/Path` |
| Input | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| Output | `/planning/scenario_planning/lane_driving/path_smoother/path` | `autoware_planning_msgs/Path` |
| Output | `/planning/scenario_planning/lane_driving/path_smoother/trajectory` | `autoware_planning_msgs/Trajectory` |

## Ported behavior

The implementation crops the path around the ego vehicle, resamples it at
1.0 m for optimization, solves the elastic-band lateral-offset box problem,
restores the BVP velocity profile and exact stop position, joins the remaining
path, and resamples the final output at 0.5 m. A backward path is passed through,
matching the upstream module's unsupported-backward-path policy.

The mathematical objective and parameter defaults follow the upstream elastic
band implementation. The ROS 1 port uses SciPy L-BFGS-B for the same convex
box-constrained quadratic objective because the ROS 1 workspace does not ship
the upstream C++ OSQP interface. This solver substitution means numeric points
are expected to match within tolerance, not bit-for-bit.

## Validation

The references were captured from the original ROS 2 Autoware Universe 0.51.0
path smoother in the Planning Simulator environment. Both tests replay the exact
ROS 2 input path and odometry into the ROS 1 node.

### Stop-line route

This long route checks straight-path preservation, 0.5 m output spacing, bound
copying, and preservation of the BVP stop position.

```bash
source /opt/ros/noetic/setup.bash
source ~/AJOUNICE2026/repo/base_ws/devel/setup.bash
roslaunch autoware_path_smoother path_smoother_stop_line_validation.launch
```

### Crosswalk route

This curved route checks smoothing around changing curvature and preservation
of a stop point close to the ego vehicle.

```bash
roslaunch autoware_path_smoother path_smoother_crosswalk_validation.launch
```

Measured results against the ROS 2 reference:

| Scenario | Geometry RMSE | Maximum geometry error | Length error | Stop error | Velocity RMSE |
| --- | ---: | ---: | ---: | ---: | ---: |
| Stop line 103624 | 0.0094 m | 0.0556 m | 0.0021 m | 0.0000 m | 0.0000 m/s |
| Crosswalk 201266 | 0.0099 m | 0.0260 m | 0.0326 m | 0.0185 m | 0.0000 m/s |

RViz colors:

- gray: BVP input path
- cyan thick line: ROS 2 Planning Simulator reference
- magenta thin line: ROS 1 port output
- cyan/magenta side lines: the same outputs shifted laterally for inspection
- cyan/magenta spheres: each output's first zero-velocity point

The report checks bidirectional geometry RMSE and maximum error, start/end
positions, path length, velocity profile, first stop position, and exact bound
copying. JSON outputs are written under
`local_autoware_planning_sim/results/path_smoother/`, outside the Git repository.

## Integrated launch

```bash
roslaunch bringup mission_bpp_bvp_path_smoother_pipeline.launch \
  map_file:=$HOME/AJOUNICE2026/lanelet2_map_v44.osm \
  map_projector_type:=MGRS
```
