# BVP Advanced Modules ROS 1 Validation

This document covers the ROS 1 ports and Planning Simulator reference tests for
`detection_area`, `merge_from_private`, `virtual_traffic_light`,
`occlusion_spot`, and `run_out`.

## Validation method

1. Start Autoware Planning Simulator 0.51.0 with the K-City v44 map or the
   module-specific validation map.
2. Publish the same route, odometry, map rule, pointcloud, occupancy grid,
   V2X state, and predicted objects required by the scenario.
3. Capture the ROS 2 module output and planning factors as reference JSON.
4. Replay the captured inputs to the ROS 1 module.
5. Compare the stop decision, stop position, geometry, and velocity profile.
6. Display the ROS 2 and ROS 1 profiles separately in RViz.

The RViz paths are shifted only for readability:

- ROS 2 Planning Simulator: cyan outline, shifted 1.2 m left;
- ROS 1 port: magenta outline, shifted 1.2 m right;
- velocity: red is 0 m/s, yellow is intermediate, cyan is maximum;
- colored spheres: first zero-velocity positions.

## Scenarios and results

| Module | Scenario | Reference behavior | Stop error | Velocity RMSE | Result |
|---|---|---|---:|---:|---|
| Detection Area | pointcloud inside detection polygon | Stop before regulatory stop line | 0.00005 m | 0.00006 m/s | PASS |
| Merge From Private | lane 200208 private to lane 200166 public | Mandatory stop before first conflict lane 200295 | 0.121 m | 0.126 m/s | PASS |
| Virtual Traffic Light | ego passed start line, no infrastructure approval | Publish REQUESTING and stop at VTL stop line | 0.00004 m | 0.00004 m/s | PASS |
| Occlusion Spot | unknown occupancy grid value 50 beside route | Stop at current ego state and continue with 1 m/s safe profile | 0.022 m | 0.012 m/s | PASS |
| Run Out | pedestrian crosses ordinary road at 0.5 m/s | Stop before predicted footprint collision | 0.069 m | 0.000 m/s | PASS |

Run Out is spatially aligned before comparison. The Motion Velocity Planner
trajectory begins 4.000 m after the BVP input path, so comparing raw station
numbers without aligning the path origins would report the wrong physical stop
position.

## Scenario details

### Detection Area

1. The route passes lanelet 200208 and its Detection Area regulatory element.
2. Nine pointcloud returns are placed inside the detection polygon.
3. The module checks the pointcloud after the configured response delay.
4. Both planners insert zero velocity at the same stop line.

RViz shows the orange detection polygon, red stop line, and red point cluster.

### Merge From Private

1. The route enters private lanelet 200208.
2. Its following lanelet 200166 is a public road.
3. The vehicle footprint first overlaps conflict lanelet 200295.
4. Both planners insert the mandatory stop 3 m before the conflict boundary.

RViz fills private lane 200208 in purple, public lane 200166 in cyan, and the
first conflict lane 200295 in orange.

### Virtual Traffic Light

1. The ego is between virtual traffic light start and stop lines.
2. Infrastructure state for instrument 9901401 has `approval=false`.
3. Both planners publish a REQUESTING command.
4. Both planners stop at the virtual stop line and wait for approval.

RViz labels the instrument, start, stop, and end lines independently.

### Occlusion Spot

1. The occupancy grid covers the route and all cells have unknown value 50.
2. A possible pedestrian collision is found at the first searchable lateral
   position.
3. The stopped ego is anchored at 0 m/s.
4. The forward profile rises with the jerk-limited restart curve and remains
   capped at 1 m/s through the processing horizon.

RViz shows the unknown-grid boundary and the two speed profiles.

### Run Out

1. A pedestrian starts 4 m to the side of an ordinary road segment. A
   crosswalk is not used because Autoware delegates crosswalk pedestrians to
   the Crosswalk module.
2. The predicted path crosses the ego trajectory at 0.5 m/s after about 8 s.
3. The collision persists longer than the stop on-time buffer.
4. Both planners stop before the collision footprint using the configured
   vehicle-front and distance buffers.

RViz shows the pedestrian body and its orange predicted crossing path.

## RViz commands

Build and source the workspace first:

```bash
cd ~/AJOUNICE2026/repo/base_ws
catkin_make
source devel/setup.bash
```

Open one comparison at a time:

```bash
roslaunch behavior_velocity_planner bvp_detection_area_comparison.launch
roslaunch behavior_velocity_planner bvp_merge_from_private_comparison.launch
roslaunch behavior_velocity_planner bvp_virtual_traffic_light_comparison.launch
roslaunch behavior_velocity_planner bvp_occlusion_spot_comparison.launch
roslaunch behavior_velocity_planner bvp_run_out_comparison.launch
```

## Porting scope

These tests validate the selected competition-facing scenario surfaces, not
bit-identical execution of every C++ branch in Autoware Universe. The ROS 1
implementation retains the Autoware message contracts and decision shape but
uses Python geometry and state machines.

The largest architectural difference is Run Out. Current Autoware implements
it as a Motion Velocity Planner plugin and calculates ego arrival times from a
smoothed `Trajectory`. The ROS 1 adapter remains in this BVP package and uses a
limited-acceleration arrival-time model because the ROS 1 pipeline does not yet
provide the same trajectory smoother at this stage.
