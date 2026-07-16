# Behavior Velocity Planner ROS 1 Port

This package is the integrated ROS 1 Behavior Velocity Planner runtime. It
keeps the Autoware execution shape: a `PathWithLaneId` enters the planner,
configured scene-module managers modify velocity in sequence, and an
`autoware_planning_msgs/Path` is published.

## Implemented scope

- reusable scene-module manager interface in the internal
  `behavior_velocity_planner_common` Python package;
- module loading through the `~launch_modules` and `~module_plugins` parameters;
- Lanelet2 OSM and raw-XML `LaneletMapBin` loading;
- Autoware-shaped odometry, object, pointcloud, and traffic-signal inputs;
- Path, StopReasonArray, PlanningFactorArray, and debug-marker outputs;
- Stop Line module;
- Speed Bump module with map-polygon parsing, height-based speed calculation,
  vehicle-footprint margins, planning factors, and debug markers.
- Traffic Light module with Lanelet2 regulatory-element parsing, circle and
  directional-arrow handling, signal timeout/hysteresis, yellow pass judgment,
  restart suppression, planning factors, and debug markers.
- Crosswalk module with Lanelet2 polygon parsing, predicted-object collision,
  TTC/TTV yield judgment, stop-position selection, and map slowdown support.
- Walkway module with mandatory stop and `APPROACH -> STOP -> SURPASSED`
  release state machine.
- Intersection module with Lanelet2 conflict topology, predicted-object TTC,
  stuck/yield-stuck decisions, pass judgment, traffic priority, and occupancy
  grid occlusion states.
- Blind Spot module with left/right turn activation, blind-side attention and
  ego conflict areas, VRU predicted-path TTC, debounce, braking/pass judgment,
  planning factors, and debug markers.
- No Stopping Area module with Lanelet2 regulatory-area parsing, stopped-object
  and path-stop detection, clear-time hysteresis, pass judgment, planning
  factors, and debug markers.
- No Drivable Lane module with tagged-lane intersection handling,
  approach/inside/stopped states, planning factors, and debug markers.
- Detection Area module with regulatory polygon and stop-line parsing,
  pointcloud/object filtering, stop hold, pass judgment, and debug markers.
- Merge From Private module with private-to-public transition detection,
  vehicle-footprint conflict search, mandatory stop, and planning factors.
- Virtual Traffic Light module with start/stop/end-line handling, V2X state
  matching, infrastructure REQUESTING/PASSING/FINALIZING commands, and stops.
- Occlusion Spot module with unknown occupancy-grid search, vehicle-front
  collision conversion, 0.1 m interpolation, and safe restart profiles.
- Run Out adapter with predicted-path collision timing, stop/slowdown debounce,
  footprint buffers, planning factors, and expected ego arrival-time modeling.

Configured modules are represented by real plugins rather than pass-through
stubs. The default launch enables the implemented road-driving BVP modules.

## Run

```bash
roslaunch behavior_velocity_planner behavior_velocity_planner.launch \
  map_file:=/path/to/lanelet2_map.osm \
  map_projector_type:=MGRS
```

The default launch connects directly to the production BPP output:

```text
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
  -> behavior_velocity_planner
  -> /planning/scenario_planning/lane_driving/behavior_planning/path
```

Mission Planner, BPP, and BVP can be started together with:

```bash
roslaunch bringup mission_bpp_bvp_pipeline.launch \
  map_file:=/path/to/lanelet2_map.osm \
  map_projector_type:=MGRS
```

## Module plugin contract

Each plugin derives from `SceneModuleManagerInterface` and implements:

```python
modify_path_velocity(path_with_lane_id, output_path, planner_data)
```

The plugin modifies `output_path` in place and returns a `SceneModuleResult`
containing stop reasons, planning factors, RViz markers, and a concise status.
The common `PlannerData` carries map, odometry, objects, pointclouds, traffic
signals, and planning time.

## Package layout

The ROS-facing implementation is kept in one catkin package, matching the
repository layout used by Mission Planner and Behavior Path Planner:

```text
behavior_velocity_planner/
  config/
  docs/
  launch/
  scripts/
  src/
    behavior_velocity_planner_common/
    behavior_velocity_blind_spot_module/
    behavior_velocity_crosswalk_module/
    behavior_velocity_detection_area_module/
    behavior_velocity_intersection_module/
    behavior_velocity_merge_from_private_module/
    behavior_velocity_no_drivable_lane_module/
    behavior_velocity_no_stopping_area_module/
    behavior_velocity_occlusion_spot_module/
    behavior_velocity_run_out_module/
    behavior_velocity_speed_bump_module/
    behavior_velocity_stop_line_module/
    behavior_velocity_traffic_light_module/
    behavior_velocity_virtual_traffic_light_module/
    behavior_velocity_walkway_module/
  test/
  validation/
```

The subdirectories under `src/` are Python module boundaries, not separate ROS
packages. Future BVP scene modules can be added there without scattering
catkin packages across `base_ws/src/planning`.

Restricted-area module scenarios, Planning Simulator comparison results, and
RViz commands are documented in `BVP_RESTRICTED_AREA_VALIDATION.md`.

Detection Area, Merge From Private, Virtual Traffic Light, Occlusion Spot, and
Run Out scenarios are documented in `BVP_ADVANCED_MODULES_VALIDATION.md`.
