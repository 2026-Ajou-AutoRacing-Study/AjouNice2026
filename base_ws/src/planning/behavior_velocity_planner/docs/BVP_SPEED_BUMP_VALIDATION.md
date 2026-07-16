# BVP Speed Bump Differential Validation

## Purpose

This validation follows the same differential method as the Stop Line test.
ROS 2 Autoware Planning Simulator processes a fixed K-City scenario first. The
captured BPP `PathWithLaneId`, route, and odometry are then replayed into the ROS
1 BVP port, and both outputs are compared numerically and in RViz.

## Implemented behavior

The ROS 1 module implements the current Autoware Speed Bump behavior used by this
scenario:

- parse a Lanelet2 `speed_bump` regulatory element and its polygon;
- activate the module only when the path uses a linked lanelet and intersects the
  polygon;
- calculate the slow speed from `height`, or use the optional
  `slow_down_speed` annotation in km/h;
- account for the vehicle front and rear offsets when placing the slow interval;
- insert slow-start and slow-end path points and cap velocity only inside that
  interval;
- publish a ROS 1 planning factor, debug status, and RViz markers.

The map representation follows the
[Autoware Lanelet2 Speed Bump specification](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/extra_regulatory_elements.md).

## Scenario

1. Ego starts on lanelet `200740` at `(2463.887, 24641.668)`.
2. The route follows `200740 -> 200739 -> 200958 -> 200610`.
3. Validation lanelet `200739` contains Speed Bump regulatory element
   `901200739` and polygon `900200739`.
4. The polygon has `height=0.15 m` and does not specify a
   `slow_down_speed` override.
5. Both planners receive the same 69-point BPP path and the same ego state.
6. The module inserts slow-start and slow-end points around the polygon and caps
   the velocity to `2.224 m/s` between them.
7. ROS 2 and ROS 1 output geometry, control positions, target speed, and the
   complete velocity profile are compared.

The original K-City OSM has no Speed Bump regulatory element. The generator
therefore creates a validation-only map copy named
`kcity_speed_bump_validation_map`. It adds the polygon and regulatory relation to
lanelet `200739`; the original `kcity_map` remains unchanged.

## Parameters

| Parameter | Value |
| --- | ---: |
| `slow_start_margin` | 1.0 m |
| `slow_end_margin` | 1.0 m |
| `min_height` | 0.05 m |
| `max_height` | 0.30 m |
| `min_speed` | 1.39 m/s |
| `max_speed` | 2.78 m/s |
| `base_link_to_front` | 3.79 m |
| `base_link_to_rear` | 1.10 m |

For this 0.15 m bump, the height interpolation produces `2.224 m/s`. The
slow-start station is the polygon entry minus the front offset and start margin;
the slow-end station is the polygon exit plus the rear offset and end margin.

## Result

The captured comparison result is `PASS`.

| Metric | ROS 2 | ROS 1 | Error |
| --- | ---: | ---: | ---: |
| Input/output points | 69 / 71 | 69 / 71 | - |
| Slow-start station | 54.038893 m | 54.038946 m | 0.000053 m |
| Slow-end station | 62.774949 m | 62.775009 m | 0.000059 m |
| Target velocity | 2.224000 m/s | 2.224000 m/s | 0.000000 m/s |
| Output geometry RMSE | - | - | 0.000000 m |
| Output geometry maximum | - | - | 0.000000 m |
| Velocity profile RMSE | - | - | 0.000007 m/s |
| Velocity profile maximum | - | - | 0.000087 m/s |

The current Planning Simulator preset disables the Speed Bump module by default,
so the reference run explicitly sets `launch_speed_bump_module:=true`. In this
Autoware release, the module's planning-factor array is empty while its official
debug markers are active. The reference capture therefore obtains the two
control positions from those debug markers and obtains the target speed from the
output path.

This result validates the fixed scenario above. It is not evidence that every
map geometry, vehicle model, or parameter combination is already equivalent.

## RViz

```bash
cd /home/vic/AJOUNICE2026/repo
source /opt/ros/noetic/setup.bash
source /home/vic/AJOUNICE2026/local_ros_deps/source_lanelet2.sh
source base_ws/devel/setup.bash
roslaunch behavior_velocity_planner bvp_speed_bump_comparison.launch
```

Legend:

- upper path with cyan outline: ROS 2 Planning Simulator result, displayed with
  a `+1.2 m` visualization-only offset;
- lower path with magenta outline: ROS 1 port result, displayed with a `-1.2 m`
  visualization-only offset;
- speed fill color: red at `0.0 m/s`, yellow at half the scenario maximum, and
  cyan at the maximum velocity;
- thin gray path: the common BPP input at its real map position;
- orange polygon: Speed Bump `901200739`;
- green sphere: slow-start control point;
- pink sphere: slow-end control point;
- white arrow: fixed ego pose;
- yellow sphere: fixed route goal.

The lateral offsets are only for visibility. All numeric comparison uses the
unmodified coordinates stored in JSON.

## Generated artifacts

Validation-only files are kept outside the Git repository:

```text
/home/vic/AJOUNICE2026/local_autoware_planning_sim/data/maps/kcity_speed_bump_validation_map/lanelet2_map.osm
/home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bvp_speed_bump/kcity_901200739.reference.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_speed_bump/kcity_901200739.ros1.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_speed_bump/kcity_901200739.report.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_speed_bump/kcity_901200739.rviz.png
```

## Reproduction

Create the validation map:

```bash
python3 /home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/create_kcity_speed_bump_validation_map.py
```

Start Planning Simulator with the validation map and explicitly enable the
module:

```bash
cd /home/vic/AJOUNICE2026/local_autoware_planning_sim/compose
AUTOWARE_MAP_NAME=kcity_speed_bump_validation_map \
  docker compose -f planning-simulator.compose.yaml run --rm \
  --name autoware_speed_bump_validation planning-simulator \
  ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/aw/autoware_data/maps/kcity_speed_bump_validation_map \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
  launch_speed_bump_module:=true rviz:=false
```

Publish the fixed ROS 2 scenario and capture the reference:

```bash
docker exec autoware_speed_bump_validation bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /opt/autoware/setup.bash; \
   python3 /home/aw/planning_sim_tools/setup_ros2_bvp_speed_bump.py'

docker exec autoware_speed_bump_validation bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /opt/autoware/setup.bash; \
   python3 /home/aw/planning_sim_tools/capture_ros2_bvp_speed_bump.py \
   --output /home/aw/planning_sim_bags/bvp_speed_bump/kcity_901200739.reference.json'
```

Start the isolated ROS 1 replay node:

```bash
cd /home/vic/AJOUNICE2026/repo
source /opt/ros/noetic/setup.bash
source /home/vic/AJOUNICE2026/local_ros_deps/source_lanelet2.sh
source base_ws/devel/setup.bash
roslaunch behavior_velocity_planner bvp_speed_bump_replay.launch
```

Capture and compare the ROS 1 result in another ROS 1 terminal:

```bash
python3 /home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/replay_capture_ros1_bvp_speed_bump.py \
  --reference /home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bvp_speed_bump/kcity_901200739.reference.json \
  --output /home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_speed_bump/kcity_901200739.ros1.json

python3 /home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/compare_bvp_speed_bump.py \
  --reference /home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bvp_speed_bump/kcity_901200739.reference.json \
  --candidate /home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_speed_bump/kcity_901200739.ros1.json \
  --report /home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_speed_bump/kcity_901200739.report.json
```
