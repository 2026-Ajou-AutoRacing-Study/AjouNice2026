# Behavior Path Planner Modules MVP

This package keeps the Autoware Behavior Path Planner shape in ROS 1 by running
each BPP module as a small path-in/path-out node. The current implementation is
an MVP: it preserves topic flow and provides simple geometric behavior so each
module can be launched, tested, and later replaced with the full Autoware logic.

## Pipeline

| Order | Node | Main role in this MVP |
| --- | --- | --- |
| 1 | `lane_following_bpp.py` | Creates the first `PathWithLaneId` from map centerlines when available, otherwise route fallback. |
| 2 | `lane_change_bpp.py` | Detects the current/preferred lane from the route and map, then connects both centerlines with an S-curve. External lateral offset remains as a fallback. |
| 3 | `static_obstacle_avoidance_bpp.py` | Tracks and filters stopped objects, expands Lanelet2 bounds, generates constrained avoidance/return shifts, and gates them with directional RTC. |
| 4 | `dynamic_obstacle_avoidance_bpp.py` | Slows the path near the nearest moving object and optionally nudges laterally. |
| 5 | `start_planner_bpp.py` | Creates a pull-out style merge from a configured start lateral offset. |
| 6 | `goal_planner_bpp.py` | Creates a pull-over style final goal offset and publishes modified goal state. |
| 7 | `side_shift_bpp.py` | Applies an external lateral offset request for remote side shift. |

## Shared Node Contract

All modules after lane following use this interface:

| Direction | Topic | Type |
| --- | --- | --- |
| Input | `~input/path` | `autoware_internal_planning_msgs/PathWithLaneId` |
| Output | `~output/path` | `autoware_internal_planning_msgs/PathWithLaneId` |
| Debug | `~debug/path` | `nav_msgs/Path` |
| Debug | `~debug/status` | `std_msgs/String` |

Obstacle modules also subscribe to `~input/objects`
(`autoware_perception_msgs/PredictedObjects`) and publish `~debug/avoidance`
(`tier4_planning_msgs/AvoidanceDebugMsgArray`).

## Full Chain Launch

```bash
cd /home/vic/AJOUNICE2026/repo/base_ws
source devel/setup.bash
roslaunch behavior_path_planner behavior_path_planner_mvp.launch
```

The top-level launch file is a thin ROS 1 wrapper. The BPP-specific launch body
lives in `launch/behavior_path_planner_mvp_impl.launch`.

The final path is published to:

```text
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
```

Intermediate paths are published under `/planning/bpp/...`, and RViz-friendly
debug paths are published under `/planning/debug/bpp/...`.

## Current Limits

Lane following and normal lane change can read Lanelet2 OSM geometry through
`~map_file`. Lane change has obstacle-free and blocked-object K-City
differential baselines. Static Obstacle Avoidance has manual/automatic RTC
baselines against Planning Simulator and shares the same engine between its
standalone and integrated nodes. The common RTC layer is connected to Lane
Change and Static Obstacle Avoidance. Dynamic avoidance, start planner, goal
planner, and side shift remain lightweight ROS1 surfaces for incremental
verification.
