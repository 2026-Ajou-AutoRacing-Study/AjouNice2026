# R002 full-source candidate boundary

This candidate is a complete-source copy from the frozen ROS2 package.  It is
the only writable target for R002 rendering and does not alter the original
package or the official AW149 run.

## Source inclusion

- Build, manifest, plugin, and parameter configuration files.
- Every source unit under `src/`, including the boundary checker integration,
  slow-down interpolator, debug output, parameter model, and utility code.
- No R001 empty-result implementation is included as an implementation
  substitute.

## Required R002 evidence

1. Catkin build with every retained source unit.
2. ROS1 callback retention and freshness gates for predicted trajectory,
   control command, steering report, operation mode, and route.
3. Route UUID/goal reset behavior.
4. Positive boundary-departure slowdown plus no-departure control oracle.

Until all four hold, this remains a candidate only and cannot affect the
official 94/149 counter.

## First render evidence

- The complete-source candidate now configures as a catkin target with the
  staged boundary-checker, planner, trajectory, Lanelet, utility, and vehicle
  information provider prefixes.
- ROS1 generated-message headers for all five transport inputs were verified
  and substituted in `type_alias.hpp`.
- The first compile reached all retained core units and exposed the next
  genuine render seam: ROS2 `msg` namespaces, `rclcpp` node/timer/parameter
  APIs, polling-subscriber types, publisher ownership, and time-duration
  APIs. These are not hidden by dropping source units or returning an empty
  planning result.
- R002 now owns ROS1 subscribers for all five required topics. Their callbacks
  retain the latest ROS1 `ConstPtr`; no polling facade or invented default
  message is used. The route callback intentionally retains the last received
  route, which is the ROS1 representation of the source durable route input.
