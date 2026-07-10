# Autoware ROS1 Message Porting Table

The Autoware ROS 2 planning-related custom messages are ported as ROS 1 catkin
message packages under `base_ws/src/msgs`. The source contracts are from:

- `docs/planning_reference/modules/specs/behavior_path_planner.md`
- `docs/planning_reference/interfaces/messages.csv`
- `docs/planning_reference/interfaces/contracts.csv`

`behavior_path_planner` is now kept as the BPP Python/launch package. Message types
use original Autoware-style package names such as `autoware_planning_msgs`,
`autoware_internal_planning_msgs`, `tier4_planning_msgs`, and `tier4_rtc_msgs`.

## ROS 2 To ROS 1 Type Rules

| ROS 2 source type | ROS 1 definition used here | Reason |
| --- | --- | --- |
| `builtin_interfaces/Time` | `time` | ROS 1 native time primitive |
| `builtin_interfaces/Duration` | `duration` | ROS 1 native duration primitive |
| `unique_identifier_msgs/UUID` | `unique_identifier_msgs/UUID` | Keep UUID payload without adding a ROS 2-only dependency |
| Autoware package prefix | Same ROS 1 package name where possible | Keep message names close to the original Autoware contracts |
| ROS 2 default values, e.g. `bool use_constraints false` | Plain field, e.g. `bool use_constraints` | ROS 1 `.msg` does not support field defaults |
| `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs` | Existing ROS 1 packages | No custom port needed |

## BPP Runtime Messages

| BPP topic role | Autoware ROS 2 type | ROS 1 type in this package | Notes |
| --- | --- | --- | --- |
| input route | `autoware_planning_msgs/LaneletRoute` | `autoware_planning_msgs/LaneletRoute` | Uses `unique_identifier_msgs/UUID`, `LaneletSegment`, `LaneletPrimitive` |
| route segment | `autoware_planning_msgs/LaneletSegment` | `autoware_planning_msgs/LaneletSegment` | Package-local nested type |
| lanelet primitive | `autoware_planning_msgs/LaneletPrimitive` | `autoware_planning_msgs/LaneletPrimitive` | Preserves `id` and `primitive_type` |
| input vector map | `autoware_map_msgs/LaneletMapBin` | `autoware_map_msgs/LaneletMapBin` | Binary lanelet map payload kept as `uint8[] data` |
| input scenario | `autoware_internal_planning_msgs/Scenario` | `autoware_internal_planning_msgs/Scenario` | String constants preserved |
| external velocity limit | `autoware_internal_planning_msgs/VelocityLimit` | `autoware_internal_planning_msgs/VelocityLimit` | Timestamp converted to ROS 1 `time` |
| velocity constraints | `autoware_internal_planning_msgs/VelocityLimitConstraints` | `autoware_internal_planning_msgs/VelocityLimitConstraints` | Same scalar fields |
| input perception | `autoware_perception_msgs/PredictedObjects` | `autoware_perception_msgs/PredictedObjects` | Includes nested object, kinematics, shape, classification |
| predicted object | `autoware_perception_msgs/PredictedObject` | `autoware_perception_msgs/PredictedObject` | Uses `unique_identifier_msgs/UUID` |
| object kinematics | `autoware_perception_msgs/PredictedObjectKinematics` | `autoware_perception_msgs/PredictedObjectKinematics` | Uses ROS 1 `geometry_msgs` covariance types |
| predicted path | `autoware_perception_msgs/PredictedPath` | `autoware_perception_msgs/PredictedPath` | Duration converted to ROS 1 `duration` |
| object class | `autoware_perception_msgs/ObjectClassification` | `autoware_perception_msgs/ObjectClassification` | Constants preserved |
| object shape | `autoware_perception_msgs/Shape` | `autoware_perception_msgs/Shape` | Uses ROS 1 `geometry_msgs/Polygon` and `Vector3` |
| input traffic lights | `autoware_perception_msgs/TrafficLightGroupArray` | `autoware_perception_msgs/TrafficLightGroupArray` | Timestamp converted to ROS 1 `time` |
| traffic light group | `autoware_perception_msgs/TrafficLightGroup` | `autoware_perception_msgs/TrafficLightGroup` | Includes current elements and predictions |
| traffic light element | `autoware_perception_msgs/TrafficLightElement` | `autoware_perception_msgs/TrafficLightElement` | Color, shape, status constants preserved |
| predicted traffic light state | `autoware_perception_msgs/PredictedTrafficLightState` | `autoware_perception_msgs/PredictedTrafficLightState` | String constants preserved |
| input lateral offset | `tier4_planning_msgs/LateralOffset` | `tier4_planning_msgs/LateralOffset` | Timestamp converted to ROS 1 `time` |
| operation mode | `autoware_adapi_v1_msgs/operation_mode/OperationModeState` | `autoware_adapi_v1_msgs/OperationModeState` | Mode constants and availability flags preserved |
| output path | `autoware_internal_planning_msgs/PathWithLaneId` | `autoware_internal_planning_msgs/PathWithLaneId` | Main BPP output for BVP input |
| path point with lane ids | `autoware_internal_planning_msgs/PathPointWithLaneId` | `autoware_internal_planning_msgs/PathPointWithLaneId` | Uses `autoware_planning_msgs/PathPoint` |
| path point | `autoware_planning_msgs/PathPoint` | `autoware_planning_msgs/PathPoint` | Velocity and final-point fields preserved |
| output turn command | `autoware_vehicle_msgs/TurnIndicatorsCommand` | `autoware_vehicle_msgs/TurnIndicatorsCommand` | Timestamp converted to ROS 1 `time` |
| output hazard command | `autoware_vehicle_msgs/HazardLightsCommand` | `autoware_vehicle_msgs/HazardLightsCommand` | Timestamp converted to ROS 1 `time` |
| output modified goal | `autoware_planning_msgs/PoseWithUuidStamped` | `autoware_planning_msgs/PoseWithUuidStamped` | Uses `unique_identifier_msgs/UUID` |
| output reroute availability | `tier4_planning_msgs/RerouteAvailability` | `tier4_planning_msgs/RerouteAvailability` | Timestamp converted to ROS 1 `time` |
| debug avoidance array | `tier4_planning_msgs/AvoidanceDebugMsgArray` | `tier4_planning_msgs/AvoidanceDebugMsgArray` | Debug-only BPP output |
| debug avoidance item | `tier4_planning_msgs/AvoidanceDebugMsg` | `tier4_planning_msgs/AvoidanceDebugMsg` | Debug-only BPP output |

## Existing ROS 1 Messages Used Directly

| BPP topic role | ROS 2 type in Autoware docs | ROS 1 type to use |
| --- | --- | --- |
| input odometry | `nav_msgs/Odometry` | `nav_msgs/Odometry` |
| input acceleration | `geometry_msgs/AccelWithCovarianceStamped` | `geometry_msgs/AccelWithCovarianceStamped` |
| input occupancy grid map | `nav_msgs/OccupancyGrid` | `nav_msgs/OccupancyGrid` |
| input costmap | `nav_msgs/OccupancyGrid` | `nav_msgs/OccupancyGrid` |
| debug markers | `visualization_msgs/MarkerArray` | `visualization_msgs/MarkerArray` |
