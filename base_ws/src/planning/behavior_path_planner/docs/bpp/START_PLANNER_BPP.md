# Start Planner ROS 1 Port

## Implemented Behavior

The module handles a vehicle stopped laterally away from the route centerline. It creates a
forward shift pull-out, checks stopped and moving objects, publishes Start Planner RTC state,
and applies the candidate automatically when RTC auto mode is enabled.

The shift duration uses Autoware Core's jerk and lateral-acceleration calculation. Pull-out
distance also considers longitudinal acceleration and maximum path curvature.

## K-City Scenario

1. The route follows lanelet `200806` toward the fixed goal.
2. Ego is stopped 1.20 m to the right of the route centerline.
3. No blocking object is present.
4. ROS 2 and ROS 1 generate a leftward forward shift to the centerline.
5. Both publish Start Planner RTC module `6`, `safe=true`, `requested=false`, state `RUNNING`,
   and auto mode enabled.

Result: lateral-profile RMSE `0.019 m`, maximum error `0.103 m`, RTC finish-distance error
`0.636 m`.

## RViz

```bash
roslaunch behavior_path_planner bpp_start_planner_comparison.launch \
  reference_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bpp_start_planner/kcity_forward_shift.reference.json \
  candidate_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bpp_start_planner/kcity_forward_shift.ros1.json \
  report_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bpp_start_planner/kcity_forward_shift.report.json
```

Reverse, geometric, clothoid, and freespace pull-out are excluded with explicit false
configuration values.
