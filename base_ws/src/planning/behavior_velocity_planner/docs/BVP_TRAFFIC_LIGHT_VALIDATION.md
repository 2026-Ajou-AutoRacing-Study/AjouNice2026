# BVP Traffic Light ROS 1 Port and Validation

## Implemented behavior

The ROS 1 plugin follows the Autoware Universe 0.51.0 Traffic Light module
contract:

- parses Lanelet2 `traffic_light` regulatory elements and their `ref_line`;
- uses `autoware_perception_msgs/TrafficLightGroupArray` without a new message;
- passes a green circle or a green directional arrow matching the lane turn;
- stops for red, amber, unknown, empty, and timed-out signals;
- applies the 0.1 s stop-state hysteresis and 1.0 s signal timeout;
- places the stop point at `stop_line - stop_margin - base_link_to_front`;
- applies Autoware's jerk-limited yellow pass judgment;
- supports restart suppression and optional V2I remaining-time prediction;
- publishes Path, StopReason, PlanningFactor, selected signal, and RViz markers.

Official references:

- <https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/>
- <https://github.com/autowarefoundation/autoware_universe/tree/0.51.0/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module>

## K-City scenario

1. The common BPP input path follows lanelets `200332 -> 200431 -> 200335 ->
   200436 -> 200221 -> 200401 -> 200207`.
2. Lanelet `200335` contains traffic-light stop line `103527`.
3. The physical signal heads use regulatory IDs `201318`, `201354`, and
   `201359`; the port selects `201318` as the representative module.
4. The exact same route, odometry, input `PathWithLaneId`, and traffic-signal
   state are supplied to ROS 2 Planning Simulator and the ROS 1 port.
5. RED and timeout must stop before the line. GREEN must retain the pass path.

## Differential result

| Case | ROS 2 | ROS 1 | Stop station error | Velocity RMSE | Result |
|---|---:|---:|---:|---:|---:|
| RED | stop at 56.840 m | stop at 56.840 m | 0.000 m | 0.000 m/s | PASS |
| GREEN | pass | pass | N/A | 0.000 m/s | PASS |
| timeout (2 s old) | stop at 56.840 m | stop at 56.840 m | 0.000 m | 0.000 m/s | PASS |

For RED and timeout, output-geometry RMSE is 0.000 m and maximum error is
0.002 m. GREEN geometry is identical. The ROS 2 Traffic Light planning-factor
topic was empty in this simulator run, so the primary stop verdict is derived
from the actual output velocity transition; the factor array is retained as
supporting evidence.

The isolated BVP output is a velocity-limit path, not the final dynamically
smoothed trajectory. In both implementations the command remains 4.1667 m/s
through station 55.980 m and changes to 0.0 m/s at the inserted stop point,
56.840 m. A gradual deceleration profile is generated later by Motion Velocity
Smoother, so it is intentionally outside this module-level comparison.

Generated evidence is local and is not part of the Git repository:

```text
local_autoware_planning_sim/bags/bvp_traffic_light/
local_autoware_planning_sim/results/bvp_traffic_light/
```

## RViz comparison

```bash
cd ~/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source ~/AJOUNICE2026/local_ros_deps/source_lanelet2.sh
source devel/setup.bash
roslaunch behavior_velocity_planner bvp_traffic_light_comparison.launch
```

The default view uses the RED case:

- cyan outline: ROS 2 Planning Simulator;
- magenta outline: ROS 1 port;
- inner speed line: red is 0 m/s, yellow is intermediate, cyan is maximum;
- orange line and red sphere: Lanelet2 stop line and RED signal state;
- cyan/magenta spheres: each implementation's first 0 m/s point;
- white arrow and yellow sphere: ego pose and route goal.

The two paths are shifted by `+/-1.2 m` only for readability. Numerical
comparison always uses the unshifted coordinates.
