# Dynamic Obstacle Avoidance BPP Port

This ROS1 module follows the Autoware Behavior Path Dynamic Obstacle Avoidance
contract. It keeps path points and velocities unchanged and expresses dynamic
avoidance as cutouts in `PathWithLaneId.left_bound` or `right_bound`. A
downstream path optimizer is responsible for generating the executable
geometric avoidance trajectory.

## Implemented Behavior

- Autoware-compatible target labels and parameter names
- regulated and unregulated object groups
- tangent/normal velocity projection onto the ego path
- crossing, lateral corridor, front-follow, cut-in, cut-out, behind-ego, and
  time-to-collision filters
- five-observation entry and one-observation exit hysteresis
- object footprint projection, one-meter clearance, 0.5-meter maximum ego
  lateral avoidance reserve, and per-object lateral low-pass state
- simultaneous left/right drivable-bound cutouts
- standalone and integrated BPP use of one shared engine
- reference path, candidate path, status, and avoidance debug outputs

There is no RTC approval gate. This matches the current Autoware dynamic
avoidance scene module, which reports an always-ready behavior and modifies
the drivable area rather than selecting an approved shifted path.

## ROS1 Representation Difference

ROS2 BPP carries obstacle polygons in internal `DrivableAreaInfo`. That
internal C++ structure is not part of the ROS message contract. The ROS1 port
therefore applies the equivalent polygon exclusion directly to the published
`PathWithLaneId` bounds. Point count can differ, so validation compares bound
geometry after station-based resampling rather than by array index.

## Validation Scenario

The differential scenario uses K-City lanelet `200806`, stationary ego, and a
car about 45 meters ahead and 1.8 meters left of the centerline, moving toward
ego. The exact ROS2 route, odometry, baseline path, bounds, and predicted
object are replayed into ROS1.

Current comparison result:

| Check | Result |
|---|---:|
| Path-point maximum error | 0.000 m |
| ROS2 / ROS1 left-bound cut | 1.685 / 2.024 m |
| Overlapping cut profile RMSE | 0.323 m |
| ROS2 / ROS1 cut interval | 18.5-29.5 / 18.0-29.5 m |
| Right-bound maximum change | 0.000 / 0.006 m |
| Verdict | PASS |

This validates one oncoming-left scenario. It does not cover every object
class, multiple simultaneous objects, all cut-in/out histories, or a
downstream path optimizer/controller closed loop.
