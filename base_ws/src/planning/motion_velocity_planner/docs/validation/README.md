# Motion Velocity Planner Validation

## Topic contract

The integrated ROS 1 node now uses the same planning surface as Autoware's
Motion Velocity Planner.

| Direction | Topic | ROS 1 message |
|---|---|---|
| Input | `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | `autoware_planning_msgs/Trajectory` |
| Input | `/perception/object_recognition/objects` | `autoware_perception_msgs/PredictedObjects` |
| Input | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| Input | `/planning/scenario_planning/max_velocity` | `autoware_internal_planning_msgs/VelocityLimit` |
| Output | `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory` | `autoware_planning_msgs/Trajectory` |

No `PathWithLaneId` adapter is used between Path Optimizer and MVP. The output
keeps every input point and pose, never raises an upstream velocity, and keeps
zero-velocity points inserted by BVP and Path Optimizer.

## Build

```bash
cd ~/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source devel_optimizer/setup.bash
catkin build motion_velocity_planner bringup --no-status
source devel_optimizer/setup.bash
```

## Standalone cases

```bash
roslaunch motion_velocity_planner validate_motion_velocity_planner.launch
```

The launch publishes native `Trajectory`, odometry, and object inputs for five
cases. It keeps RViz open after the case runner writes:

- `~/AJOUNICE2026/local_autoware_planning_sim/results/motion_velocity_planner/mvp_validation_results.json`
- `~/AJOUNICE2026/local_autoware_planning_sim/results/motion_velocity_planner/mvp_validation_results.csv`

| Case | Expected behavior | Observed result |
|---|---|---|
| Baseline, no object | Keep 5.0 m/s, no stop | PASS |
| Static object 2.0 m lateral | Ignore object | PASS |
| Leading object at 2.0 m/s | TTC cruise limit, no full stop | PASS, minimum 2.5 m/s |
| Slow object at 0.5 m/s | Insert stop at x=13 m | PASS |
| Static object | Insert stop at x=19 m | PASS |

For an automated run that closes all nodes when the report is ready:

```bash
roslaunch motion_velocity_planner validate_motion_velocity_planner.launch \
  launch_rviz:=false shutdown_on_result:=true
```

## K-City integration

This launch runs the complete ROS 1 chain on `lanelet2_map_v44.osm`:

```text
Mission Planner -> BPP -> BVP -> Path Smoother -> Path Optimizer -> MVP
```

```bash
roslaunch bringup kcity_path_optimizer_mvp_validation.launch
```

It automatically publishes the K-City stop-line reference input, opens RViz,
and writes a summary plus a point-by-point trajectory table:

```text
~/AJOUNICE2026/local_autoware_planning_sim/results/motion_velocity_planner/
kcity_path_optimizer_mvp.report.json
kcity_path_optimizer_mvp.points.csv
```

Each CSV row represents one trajectory point. It contains input/output pose,
quaternion, yaw, cumulative station, time from start, longitudinal and lateral
velocity, acceleration, heading rate, wheel angles, stop flags, and the
input/output differences. `TrajectoryPoint` does not contain a force field;
acceleration is the closest motion-state quantity available in this message.

Verified result:

| Check | Result |
|---|---:|
| Path Optimizer input points | 415 |
| MVP output points | 415 |
| Maximum XY difference | 0.0 m |
| Path Optimizer first stop | index 345, station 172.415 m |
| MVP first stop | index 345, station 172.415 m |
| Upstream zero points restored to motion | 0 |
| Output velocity greater than input | 0 |
| Overall | PASS |

The integrated test uses a 16.67 m/s upstream limit and a 5.0 m/s MVP limit,
so the output maximum changes from 16.67 m/s to 5.0 m/s by design.

For CI-style execution without RViz:

```bash
roslaunch bringup kcity_path_optimizer_mvp_validation.launch \
  launch_rviz:=false shutdown_on_result:=true
```

## RViz legend

| Marker | Meaning |
|---|---|
| Large gray points | Path Optimizer input trajectory |
| Small green points | High-speed MVP output points |
| Small yellow points | Mid-speed MVP output points |
| Small red points | Low-speed or stopped MVP output points |
| Large red sphere | First zero-velocity point |
| Blue arrow | Ego pose |
| Orange box | Perceived object |

The gray and colored points should overlap geometrically. Only their velocity
color is expected to change.

## Current scope

This validates the native trajectory interface and the current aggregate ROS 1
MVP logic. It is not yet full source-level parity with every Autoware MVP plugin.
The integrated node currently covers base velocity limiting, obstacle stop,
slowdown, cruise following, dynamic stop, and TTC limiting. Pointcloud,
occupancy-grid, out-of-lane, boundary-departure, and plugin-manager parity still
need separate ROS 2 reference scenarios before this package can be called a
complete Autoware port.
