# Mission Planner + BPP Validation

The production launch is `launch/mission_bpp_pipeline.launch`. It connects the
Mission Planner route output directly to the integrated Behavior Path Planner.

The validation launch creates a synthetic waypoint-route request and checks:

- Mission Planner reaches route state `SET`
- route and BPP path contain the expected lane ID
- BPP publishes at least two finite path points
- the path ends at the requested goal with zero terminal velocity
- Goal Planner and Side Shift remain disabled in the competition profile

Run it with:

```bash
roslaunch bringup mission_bpp_pipeline_minimal_test.launch
```

The validator exits with code `0` on success and stops the test launch.

The checked baseline currently produces one route segment and a 31-point BPP
path from `(0, 0)` to `(60, 0)`, with a zero terminal velocity.

## K-City Planning Simulator Comparison

The stronger differential test uses the ROS2 Planning Simulator capture as the
shared input. It sends only the captured ego pose and goal to the ROS1 Mission
Planner, lets that planner create the route, and captures the BPP path generated
from the live route topic.

The local runner and full commands are documented at:

```text
/home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/MISSION_BPP_PIPELINE_VALIDATION.md
```

Current K-City result:

| Check | Result |
| --- | ---: |
| Preferred lanelet sequence | exact match, 7 sections |
| ROS2 / ROS1 path points | 99 / 94 |
| Path length error | 0.162 m |
| Path start error | 0.207 m |
| Path goal error | 0.000 m |
| Centerline RMSE / max error | 0.121 m / 0.700 m |
| Velocity surface | exact match |
| Left/right drivable bounds | PASS / PASS |
| Overall verdict | PASS |

## Mission Planner + BPP + BVP Integration

The K-City v44 integration scenario starts from the ROS2 Planning Simulator
stop-sign capture. Unlike the isolated BVP replay, it sends only the captured
ego pose, goal, and environment inputs. ROS1 Mission Planner creates the route,
BPP creates `PathWithLaneId`, and the production BVP modifies that live path.

Headless automatic validation for both a map-regulation case and a
dynamic-object case:

```bash
roslaunch bringup kcity_mission_bpp_bvp_stop_line.launch \
  launch_rviz:=false shutdown_on_result:=true

roslaunch bringup kcity_mission_bpp_bvp_crosswalk.launch \
  launch_rviz:=false shutdown_on_result:=true
```

RViz validation:

```bash
roslaunch bringup kcity_mission_bpp_bvp_stop_line.launch

roslaunch bringup kcity_mission_bpp_bvp_crosswalk.launch
```

The validator checks the route sequence, finite BPP/BVP paths, path-geometry
preservation, the expected planning factor and stop reason, and the stop
pose/station against the ROS2 reference. The current stop-line and crosswalk
scenarios both pass. In RViz, the ROS2 speed path is offset `+1.2 m`, the live
ROS1 full-pipeline path is offset `-1.2 m`, and speed is colored red through
yellow to cyan.
