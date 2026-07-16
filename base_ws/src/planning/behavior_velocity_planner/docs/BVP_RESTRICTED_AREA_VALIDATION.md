# BVP Restricted-Area Modules

This document records the ROS 1 port and validation of Autoware's
`no_stopping_area` and `no_drivable_lane` Behavior Velocity Planner modules.

Official behavior references:

- https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/index.html
- https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module/

## Implemented behavior

### No Stopping Area

- parses Lanelet2 `no_stopping_area` regulatory elements and their polygons;
- uses an explicit `ref_line` when supplied and otherwise generates the stop
  position from the polygon entry;
- detects stopped vehicles and pre-existing zero-velocity points inside the
  prohibited area;
- keeps the stop decision for the configured clear time;
- applies dead-line and jerk/acceleration pass judgment;
- publishes the modified path, StopReason, PlanningFactor, SafetyFactor,
  status, and debug markers.

The map must connect a `regulatory_element` relation with
`subtype=no_stopping_area` to the affected road lanelet. Its `refers` member is
the prohibited polygon. A `ref_line` stop line is optional.

### No Drivable Lane

- detects route lanelets tagged `no_drivable_lane=yes`;
- calculates path entry and exit intersections with the lanelet polygon;
- stops before the lanelet while approaching;
- stops at the current ego position if planning starts inside the lanelet;
- maintains `APPROACHING`, `INSIDE_NO_DRIVABLE_LANE`, and `STOPPED` states;
- publishes the modified path, StopReason, PlanningFactor, status, and debug
  markers.

The production K-City v44 map is not modified by these tests. Dedicated copies
under `local_autoware_planning_sim/data/maps/` add the required test-only map
attributes to lanelet `200208`.

## Validation scenarios

### No Stopping Area: clear

1. The route passes through lanelet `200208` and regulatory area `9901210`.
2. No stopped object or zero-velocity path point exists inside the area.
3. The ROS 1 module reports `9901210:GO` and leaves the path unchanged.
4. Result: PASS. No active module PlanningFactor is produced.

### No Stopping Area: stuck vehicle

1. The same route and regulatory area are used.
2. A stopped car is placed inside the no-stopping polygon at path station
   `27.998 m`.
3. The module detects the car and inserts a stop before the area at
   `16.413 m`.
4. The resulting clearance to the stopped car is `11.585 m` and a
   `stuck_vehicle` PlanningFactor is produced.
5. Result: ROS 1 semantic validation PASS.

The installed ROS 2 Planning Simulator crashes in
`NoStoppingAreaModule::check_stuck_vehicles_in_no_stopping_area()` for both an
empty and a populated `PredictedObjects` input. The stable and experimental
plugins both reproduce the SIGSEGV. Therefore no valid ROS 2 output exists for
this module and a differential PASS is deliberately not claimed. The failure
record is:

```text
local_autoware_planning_sim/results/bvp_no_stopping_area/
  kcity_planning_simulator_upstream_failure.report.json
```

### No Drivable Lane: approaching

1. The input path approaches lanelet `200208`, tagged
   `no_drivable_lane=yes`.
2. Both planners stop before the restricted lanelet.
3. ROS 2 stop station: `7.778144 m`.
4. ROS 1 stop station: `7.778347 m`.
5. Stop error: `0.000204 m`; velocity RMSE: `0.000197 m/s`.
6. Result: differential validation PASS.

### No Drivable Lane: inside

1. Planning starts with the ego vehicle already inside lanelet `200208`.
2. Both planners request an immediate stop at the ego position and remain in
   the stopped state.
3. ROS 2 stop station: `8.378645 m`.
4. ROS 1 stop station: `8.378545 m`.
5. Stop error: `0.000100 m`; velocity RMSE: `0.000209 m/s`.
6. Result: differential validation PASS.

## RViz

Build and source the workspace first:

```bash
cd /home/vic/AJOUNICE2026/repo/base_ws
catkin_make
source devel/setup.bash
```

No Stopping Area:

```bash
roslaunch behavior_velocity_planner bvp_no_stopping_area_comparison.launch
```

The orange polygon is the prohibited area, the red box is the stopped car,
and the magenta outline is the ROS 1 path. Segment colors encode velocity:
red is stopped, yellow is intermediate speed, and cyan is the scenario maximum.
The overlay states that the ROS 2 reference is blocked by the upstream crash.

No Drivable Lane:

```bash
roslaunch behavior_velocity_planner bvp_no_drivable_lane_comparison.launch \
  scenario:=approaching
```

Use `scenario:=inside` for the second case. The translucent magenta area is
lanelet `200208`. The cyan outline is the ROS 2 Planning Simulator result and
the magenta outline is the ROS 1 result; they are shifted sideways only for
readability. Their colored center segments show each path's actual velocity.

## Reports

```text
local_autoware_planning_sim/results/bvp_no_stopping_area/
  kcity_clear.ros1.json
  kcity_stuck_vehicle.ros1.json
  kcity_ros1_semantic.report.json
  kcity_planning_simulator_upstream_failure.report.json

local_autoware_planning_sim/results/bvp_no_drivable_lane/
  kcity_approaching.ros1.json
  kcity_approaching.report.json
  kcity_inside.ros1.json
  kcity_inside.report.json
```
