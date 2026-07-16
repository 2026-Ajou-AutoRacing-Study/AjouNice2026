# BVP Stop Line Differential Validation

## Purpose

This validation uses a fixed K-City scenario. It does not require RViz pose clicks.
The ROS 2 Autoware Planning Simulator result is captured first, then the exact same
BPP `PathWithLaneId` and odometry are replayed into the ROS 1 BVP port.

## Scenario

1. Ego starts on lanelet `200740` at `(2463.887, 24641.668)`.
2. The route follows `200740 -> 200739 -> 200958 -> 200610`.
3. Lanelet `200739` contains map stop line way `103498`.
4. The BVP Stop Line module must insert a zero-velocity point before the line.
5. ROS 2 and ROS 1 output geometry, stop position, stop station, and velocity
   profile are compared.

The original K-City OSM contains way `103498` as `subtype=stop_line`, but it does
not connect the way to a `traffic_sign` regulatory element. Current Autoware only
creates this module for a `stop_sign` regulatory element. The generator therefore
creates a validation-only map copy named `kcity_stop_line_validation_map` and adds:

- regulatory element `900103498` with `sign_type=stop_sign`;
- `103498` as its `ref_line`;
- the regulatory element as a member of lanelet `200739`.

The original `kcity_map` is not modified.

## Result

The current captured result is `PASS`.

| Metric | Result |
| --- | ---: |
| ROS 2 input/output points | 69 / 70 |
| ROS 1 input/output points | 69 / 70 |
| ROS 2 stop station | 62.558 m |
| ROS 1 stop station | 62.558 m |
| Stop control-point position error | 0.017 m |
| Stop station error | 0.000 m |
| Output path geometry RMSE | 0.002 m |
| Output path maximum geometry error | 0.017 m |
| Velocity profile RMSE | 0.000 m/s |
| Velocity profile maximum error | 0.002 m/s |

The comparison also exposed a parameter mismatch in the initial ROS 1 port. The
ROS 1 defaults were updated to the current Autoware values:

| Parameter | Autoware-compatible value |
| --- | ---: |
| `stop_margin` | 0.0 m |
| `required_stop_duration_sec` | 1.0 s |
| `vehicle_stopped_duration_threshold` | 0.0 m/s |
| `hold_stop_margin_distance` | 2.0 m |

## RViz

```bash
cd /home/vic/AJOUNICE2026/repo
source /opt/ros/noetic/setup.bash
source /home/vic/AJOUNICE2026/local_ros_deps/source_lanelet2.sh
source base_ws/devel/setup.bash
roslaunch behavior_velocity_planner bvp_stop_line_comparison.launch
```

Legend:

- left path with cyan outline: ROS 2 Planning Simulator speed path, displayed
  with a `+1.2 m` visualization-only offset;
- right path with magenta outline: ROS 1 speed path, displayed with a `-1.2 m`
  visualization-only offset;
- speed fill color: red at `0.0 m/s`, yellow at half of the scenario maximum,
  and cyan at the maximum velocity;
- thin gray center path: common BPP input path at its real map position;
- orange: map stop line `103498`;
- source-colored sphere: each output's stop control point;
- white arrow: fixed ego pose;
- yellow sphere: fixed route goal.

`ROS2 Speed Path` and `ROS1 Speed Path` are separate RViz displays. They can be
toggled independently from the Displays panel. The lateral offsets only prevent
visual overlap; all numeric comparison uses the unmodified JSON coordinates.

The generated files are outside the Git repository:

```text
/home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bvp_stop_line/kcity_103498.reference.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_stop_line/kcity_103498.ros1.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_stop_line/kcity_103498.report.json
/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_stop_line/kcity_103498.rviz.png
```

## Reproduction

Create the validation map:

```bash
python3 /home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/create_kcity_stop_line_validation_map.py
```

Start the Planning Simulator:

```bash
AUTOWARE_MAP_NAME=kcity_stop_line_validation_map \
  /home/vic/AJOUNICE2026/local_autoware_planning_sim/scripts/run_planning_sim_cpu.sh
```

In another terminal, find the container and publish the fixed scenario:

```bash
CONTAINER_ID=$(docker ps --filter name=compose-planning-simulator-run -q | head -n 1)
docker exec "$CONTAINER_ID" bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /opt/autoware/setup.bash; \
   python3 /home/aw/planning_sim_tools/setup_ros2_bvp_stop_line.py'
```

Capture the ROS 2 reference:

```bash
docker exec "$CONTAINER_ID" bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /opt/autoware/setup.bash; \
   python3 /home/aw/planning_sim_tools/capture_ros2_bvp_stop_line.py \
   --output /home/aw/planning_sim_bags/bvp_stop_line/kcity_103498.reference.json'
```

Start the isolated ROS 1 BVP replay node:

```bash
cd /home/vic/AJOUNICE2026/repo
source /opt/ros/noetic/setup.bash
source /home/vic/AJOUNICE2026/local_ros_deps/source_lanelet2.sh
source base_ws/devel/setup.bash
roslaunch behavior_velocity_planner bvp_stop_line_replay.launch
```

Capture and compare the ROS 1 result from another ROS 1 terminal:

```bash
python3 /home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/replay_capture_ros1_bvp_stop_line.py \
  --reference /home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bvp_stop_line/kcity_103498.reference.json \
  --output /home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_stop_line/kcity_103498.ros1.json

python3 /home/vic/AJOUNICE2026/local_autoware_planning_sim/tools/compare_bvp_stop_line.py \
  --reference /home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bvp_stop_line/kcity_103498.reference.json \
  --candidate /home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_stop_line/kcity_103498.ros1.json \
  --report /home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bvp_stop_line/kcity_103498.report.json
```
