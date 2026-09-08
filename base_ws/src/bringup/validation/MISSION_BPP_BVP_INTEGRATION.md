# Archived Mission Planner + BPP + BVP Integration Validation

This page preserves historical v44 results. The referenced external ROS2
captures and local runner were never committed, and the obsolete launch
wrappers have been removed. Do not use these commands as current v61 evidence;
use `ajounice_planning_integration/morai_planning_stack.launch` and the gates in
`docs/planning/INTEGRATION-HANDOFF.md`.

## Scenarios

### Stop Line

1. The ego starts on K-City v44 lanelet `200592`.
2. The goal is on the following lanelet `200632`.
3. Mission Planner must generate the preferred sequence `200592 -> 200632`.
4. BPP must create a continuous `PathWithLaneId` from the ego to the goal.
5. Stop-sign regulatory element `201406` references stop line `103624` on
   lanelet `200592`.
6. BVP must keep the BPP geometry and set zero velocity at the stop point.
7. The stop point is compared with the ROS2 Planning Simulator capture.

### Crosswalk

1. The ego starts before K-City crosswalk lanelet `201266`.
2. Mission Planner creates the seven-section route from the captured goal.
3. BPP generates the live lane-following path from that route.
4. The captured predicted pedestrian crosses the path inside the crosswalk.
5. BVP must report `crosswalk: ... YIELD`, publish a `Crosswalk` stop reason,
   and insert a stop point before the crossing.
6. The stop point is compared with the ROS2 Planning Simulator capture.

The ROS2 reference input is not copied into the repository. It remains under
`local_autoware_planning_sim`, so it is not included in a normal repository
push.

The historical report locations were:

```text
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/integration/kcity_stop_line.report.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/integration/kcity_crosswalk.report.json
```

## RViz Legend

- Cyan outline, `+1.2 m`: ROS2 Planning Simulator reference.
- Magenta outline, `-1.2 m`: live ROS1 Mission Planner + BPP + BVP output.
- Thin gray line: live BPP input geometry before BVP velocity modification.
- Speed color: red is zero, yellow is medium, cyan is maximum velocity.
- Orange sphere: ROS2 reference stop point.
- Green arrow: start pose.
- Red arrow: goal pose.

The lateral display offsets are visualization-only. Validation uses the
unshifted coordinates.

## Pass Conditions

- Route state is `SET` and the preferred lanelet sequence matches ROS2.
- BPP and BVP paths contain finite values. Goal arrival is required only when
  the ROS2 reference path itself reaches the route goal; local-horizon
  references are evaluated over their published horizon.
- BVP does not laterally alter the BPP path.
- `stop_line` appears in the planning factors and module status.
- `StopLine` appears in the stop reasons.
- The live stop pose is within `1.5 m` of the ROS2 reference.
- The live stop station is within `2.0 m` of the ROS2 reference.

## Current Results

| Scenario | Route | BPP/BVP geometry | Stop pose error | Stop station error | Result |
| --- | --- | --- | ---: | ---: | --- |
| Stop line `103624` | exact, 2 sections | `0.000 m` | `0.000 m` | `1.894 m` | PASS |
| Crosswalk `201266` | exact, 7 sections | `0.000 m` | `0.025 m` | `0.206 m` | PASS |

The crosswalk ROS2 output is a local path ending `88.166 m` before the mission
goal. It is therefore not treated as a failed goal-arrival case.
