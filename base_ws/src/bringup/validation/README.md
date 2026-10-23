# Archived Mission Planner + BPP Validation Evidence

This directory records historical v44 validation evidence. Its former launch
wrappers depended on local files and packages that are not in this repository,
so they were removed instead of being exposed as runnable integration tests.
The production entrypoint is now
`ajounice_planning_integration/morai_planning_stack.launch`; follow
`docs/planning/INTEGRATION-HANDOFF.md` for current gates.

The validation launch creates a synthetic waypoint-route request and checks:

- Mission Planner reaches route state `SET`
- route and BPP path contain the expected lane ID
- BPP publishes at least two finite path points
- the path ends at the requested goal with zero terminal velocity
- Goal Planner and Side Shift remain disabled in the competition profile

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

The validator checks the route sequence, finite BPP/BVP paths, path-geometry
preservation, the expected planning factor and stop reason, and the stop
pose/station against the ROS2 reference. The current stop-line and crosswalk
historical records both passed; this is not a currently runnable test or v61
approval. In RViz, the ROS2 speed path is offset `+1.2 m`, the live
ROS1 full-pipeline path is offset `-1.2 m`, and speed is colored red through
yellow to cyan.

## RViz Route A/B editor

```bash
export AJOUNICE_ROOT="$(pwd -P)"
source scripts/workspace/source_aw149.sh
roslaunch bringup aw149_route_editor.launch
```

This opens the v85 map and the complete configured Route A/B without starting
a controller or the automatic route requester. The regular planning RViz keeps
its original driving view; editing is available through this separate launch.
In **Route Editor**, choose A/B and press **클릭 추가 (P)**
or the toolbar's **Route Point** tool. Clicking a road snaps a new waypoint to
its centerline, including elevation and travel direction, and inserts it in
route order. Use **Interact** to drag a waypoint, or select its list row and
press **선택 삭제**. The start is fixed; the goal can be moved but not deleted.
**되돌리기** reverses edits and **원본 복원** restores the selected route.

Original A is blue and B is green; the selected draft is orange. Edited road
connections are a standard Lanelet2 preview, not the Autoware Mission Planner's
verified route. Red indicates disconnected points. **Planning에 경로 적용**
explicitly calls `/planning/set_waypoint_route` with the selected waypoints and
goal; it needs a running planning stack and uses its current ego position.
The current planner route remains visible in the regular RViz **Mission Route**
display. Editing and saving alone issue no route request.

**저장** writes both drafts to
`planner_logs/route_editor/competition_routes.yaml` by default; **불러오기** opens
another saved draft. The editor reloads the default file on the next launch.
These are editor files with a map fingerprint: they do not overwrite or replace
the competition requester's frozen `kcity_v85_competition_route.yaml` contract.
