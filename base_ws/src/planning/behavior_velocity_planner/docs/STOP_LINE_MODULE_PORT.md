# Stop Line Module ROS 1 Port

The module implements the first Behavior Velocity Planner scene module. It:

1. finds intersections between the incoming BPP path and Lanelet2 ways tagged
   `subtype=stop_line`;
2. excludes stop lines owned by a `traffic_light` regulatory element because
   those belong to the Traffic Light module;
3. places the base-link stop pose behind the map line by
   `base_link_to_front + stop_margin`;
4. inserts an exact path point at that pose and sets all following velocities
   to zero;
5. holds the stop through `APPROACHING`, `STOPPED`, and `STARTED` states;
6. publishes `StopLine` stop reasons, planning factors, and RViz markers.

The K-City map currently contains 142 stop-line ways. Of these, 44 unique ways
are referenced by 124 traffic-light regulatory elements. With
`include_traffic_light_ref_lines=false`, these traffic-light lines are not
treated as unconditional stop signs.

## Parameters

| Name | Default | Meaning |
| --- | ---: | --- |
| `stop_margin` | 0.0 m | Clearance between the vehicle front and stop line |
| `base_link_to_front` | 3.79 m | Rear-axle base_link to vehicle front |
| `required_stop_duration_sec` | 1.0 s | Required continuous stopped duration |
| `vehicle_stopped_duration_threshold` | 0.0 m/s | Stopped-speed threshold |
| `hold_stop_margin_distance` | 2.0 m | Distance where stopped state is latched |
| `max_detection_distance` | 200 m | Forward stop-line search range |
| `include_traffic_light_ref_lines` | false | Include lines handled by Traffic Light |

The module unit tests cover exact stop-point insertion, zero velocity after the
stop pose, traffic-light line exclusion, and release after a complete stop.
