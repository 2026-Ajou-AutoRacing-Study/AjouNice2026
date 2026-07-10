# ajou_control

`ajou_control` is a ROS1 Noetic controller package for MORAI trajectory tracking. It can generate a virtual reference path for wiring checks and can follow a CSV-recorded MORAI ego trajectory using Pure Pursuit steering and longitudinal PI speed control.

This package follows MORAI-ROS `morai_msgs` conventions for simulator I/O and does not depend on ROS2 or Autoware Universe message types.

## Topics

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `/reference_path` | `nav_msgs/Path` | Published | Virtual reference path in the `map` frame |
| `/target_speed` | `std_msgs/Float32` | Published | Target speed in m/s |
| `/reference_path_marker` | `visualization_msgs/Marker` | Published | RViz line strip marker for the reference path |
| `/recorded_trajectory_path` | `nav_msgs/Path` | Subscribed | Recorded path published by `ajou_trajectory_recorder` |
| `/recorded_trajectory/target_speed` | `std_msgs/Float32` | Subscribed | Optional target speed from recorded CSV |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | Published | Final MORAI vehicle command topic |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | Subscribed | MORAI ego vehicle state topic |

## Build

```bash
cd ~/catkin_ws
catkin_make
source ~/.bashrc
```

If your `.bashrc` does not already source the workspace, run:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Run Trajectory Validation

```bash
roslaunch ajou_control trajectory_validation.launch
```

Example with a circular path:

```bash
roslaunch ajou_control trajectory_validation.launch mode:=circle radius:=20.0 target_speed_mps:=5.0
```

## Visualize in RViz

```bash
roslaunch ajou_control trajectory_validation.launch enable_rviz:=true
```

In RViz:

1. Set `Fixed Frame` to `map`.
2. Add `Path` and select `/reference_path`.
3. Add `Marker` and select `/reference_path_marker`.

## Test MORAI Command Topic

```bash
rosrun ajou_control simple_morai_cmd.py
rostopic echo /ctrl_cmd
```

The default command is intentionally safe:

- `longlCmdType = 1`
- `accel = 0.0`
- `brake = 1.0`
- `front_steer = 0.0`

## Follow Recorded MORAI Trajectory

The normal workflow is:

1. Record a manual drive with `ajou_trajectory_recorder`.
2. Publish the recorded CSV as a `nav_msgs/Path`.
3. Run this controller against that path.

If the CSV already exists, one launch file can start the path publisher, visualizer, controller, and optional validator:

```bash
roslaunch ajou_control recorded_path_stack.launch target_speed_mps:=3.0
```

With tracking validation enabled:

```bash
roslaunch ajou_control recorded_path_stack.launch target_speed_mps:=3.0 enable_validator:=true
```

Use a custom CSV:

```bash
roslaunch ajou_control recorded_path_stack.launch input_csv:=/absolute/path/to/recorded_trajectory.csv target_speed_mps:=3.0
```

For separate terminals, publish the CSV trajectory first:


```bash
roslaunch ajou_trajectory_recorder publish_trajectory.launch
```

Then run the Pure Pursuit + PI controller:

```bash
roslaunch ajou_control recorded_path_control.launch
```

Start slowly while tuning:

```bash
roslaunch ajou_control recorded_path_control.launch target_speed_mps:=5.0
```

If the vehicle turns away from the path, flip the steering sign:

```bash
roslaunch ajou_control recorded_path_control.launch target_speed_mps:=5.0 steering_sign:=-1.0
```

If the controller reports a large raw speed while the vehicle is visually slow, MORAI may be publishing velocity in km/h. Convert it to m/s:

```bash
roslaunch ajou_control recorded_path_control.launch target_speed_mps:=5.0 velocity_scale:=0.27778
```

The controller subscribes to:

| Topic | Type | Description |
| --- | --- | --- |
| `/recorded_trajectory_path` | `nav_msgs/Path` | Recorded path from CSV |
| `/recorded_trajectory/target_speed` | `std_msgs/Float32` | Target speed in m/s |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | MORAI ego vehicle status |

It publishes `/ctrl_cmd` as `morai_msgs/CtrlCmd` with throttle/brake PI speed control and Pure Pursuit front steering. The launch file uses a fixed low target speed by default. Set `use_target_speed_topic:=true` only after low-speed tracking is stable.

### Key Runtime Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `target_speed_mps` | `3.0` | Fixed low-speed target used while tuning |
| `use_target_speed_topic` | `false` | Use `/recorded_trajectory/target_speed` instead of fixed target |
| `steering_sign` | `1.0` | Flip to `-1.0` if MORAI steering sign is opposite |
| `velocity_scale` | `1.0` | Set to `0.27778` if MORAI velocity is km/h |
| `min_lookahead` | `7.0` | Minimum Pure Pursuit lookahead distance |
| `max_lookahead` | `16.0` | Maximum Pure Pursuit lookahead distance |
| `max_steer_rad` | `0.35` | Steering command clamp |

## Validate Tracking Accuracy

Run the validator while the controller is driving:

```bash
roslaunch ajou_control tracking_validation.launch
```

It compares `/Ego_topic` against `/recorded_trajectory_path`, writes a sample CSV, publishes RViz markers, and exposes time-series topics for `rqt_multi_plot`.

RViz:

- Add `Path`: `/recorded_trajectory_path`
- Add `MarkerArray`: `/validation/markers`
- The marker array overlays the target path, current ego-driven path, nearest target point, ego-to-target error line, and optional previous runs.

To overlay earlier validation runs while tuning one parameter at a time:

```bash
roslaunch ajou_control tracking_validation.launch \
  history_csvs:=/absolute/path/run_kp_025.csv,/absolute/path/run_kp_030.csv
```

Or load a batch:

```bash
roslaunch ajou_control tracking_validation.launch \
  history_glob:="$(env HOME)/catkin_ws/src/ajou_control/validation_runs/*.csv"
```

Recommended `rqt_multi_plot` topics:

| Group | Topic | Meaning |
| --- | --- | --- |
| Lateral | `/validation/lateral/lat_err` | Signed lateral error, m |
| Lateral | `/validation/lateral/heading_err` | Heading error, rad |
| Lateral | `/validation/lateral/steering_angle` | Commanded front steering angle, rad |
| Lateral | `/validation/lateral/lat_vel` | Ego lateral velocity in body frame, m/s |
| Lateral | `/validation/lateral/lat_acc` | Ego lateral acceleration in body frame, m/s^2 |
| Longitudinal | `/validation/longitudinal/long_vel` | Ego longitudinal velocity in body frame, m/s |
| Longitudinal | `/validation/longitudinal/target_vel` | Target velocity, m/s |
| Longitudinal | `/validation/longitudinal/long_vel_err` | Longitudinal velocity error, m/s |
| Longitudinal | `/validation/longitudinal/long_acc` | Ego longitudinal acceleration, m/s^2 |
| Run metric | `/validation/metrics/mean_ade` | Mean nearest-point displacement error over the run, m |
| Run metric | `/validation/metrics/fde` | Current distance to final target point, m |
| Run metric | `/validation/metrics/lateral_rms` | Lateral RMS error, m |
| Run metric | `/validation/metrics/heading_rms` | Heading RMS error, rad |
| Run metric | `/validation/metrics/speed_rms` | Longitudinal velocity RMS error, m/s |
| Run metric | `/validation/metrics/progress` | Path completion ratio |
| Run metric | `/validation/metrics/steer_saturation_rate` | Fraction of samples near steering saturation |

Console summary metrics:

| Metric | Meaning |
| --- | --- |
| `lateral` | Current signed lateral error in meters |
| `lateral_rms` | RMS lateral error over the validation run |
| `lateral_max` | Maximum absolute lateral error |
| `heading_rms` | RMS heading error in radians |
| `speed_rms` | RMS speed error in m/s |
| `progress` | Path completion ratio |
| `steer_sat` | Fraction of samples near steering saturation |
| `mean_ade` | Mean nearest-point displacement error over the validation run |
| `fde` | Current distance from ego to the final target path point |

The validator also writes a per-sample CSV:

```text
~/catkin_ws/src/ajou_control/validation_tracking.csv
```

The CSV includes the lateral validation columns (`lateral_error`, `heading_error_rad`, `steering_angle`, `lat_vel`, `lat_acc`), longitudinal validation columns (`long_vel`, `target_speed`, `long_vel_error`, `long_acc`), and run metrics (`ade`, `mean_ade`, `fde`, RMS values, steering saturation rate).

## Current Scope

Implemented:

- Virtual trajectory generator for path/topic validation.
- Recorded path tracking with Pure Pursuit steering.
- Longitudinal PI speed control publishing MORAI `CtrlCmd`.
- RViz debug markers for ego, nearest point, target point, lookahead, and cross-track error.
- Tracking validator with RViz target/current/previous-run overlays, separated lateral and longitudinal validation topics, ADE/FDE, RMS values, progress, and steering saturation metrics.

The skeleton scripts are kept only as reference placeholders; use `pure_pursuit_pi_controller.py` and `tracking_validator.py` for active testing.

## Generate Trajectory from MORAI MGeo Centerlines

The current MGeo centerline source is:

```text
/mnt/c/Users/lenovo/Downloads/link_set.json
```

Before using MGeo as a trajectory source, verify that MORAI world coordinates and RViz `map` coordinates are aligned 1:1:

```bash
roslaunch ajou_control mgeo_alignment_check.launch enable_rviz:=true
```

The launch file currently applies the MORAI-local to MGeo-local visual alignment offsets by default:

```text
offset_x = 0.0
offset_y = 560.0
offset_z = -28.5
offset_yaw_deg = 0.0
ego_offset_x = -134.24
ego_offset_y = -947.36
ego_offset_z = -2.38
```

These defaults match the observed MORAI ego marker against the current `link_set.json`. Override them only when the map asset or MORAI map origin changes.

In RViz, set `Fixed Frame` to `map`, then add:

- `MarkerArray`: `/mgeo_alignment/markers`

The checker overlays local MGeo centerlines, estimated lane-edge lines from each link width, the current MORAI ego pose, the nearest centerline point, ego heading, centerline heading, and the ego-to-centerline error line. It also publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/mgeo_alignment/nearest_distance` | `std_msgs/Float32` | Distance from ego to nearest transformed MGeo centerline point |
| `/mgeo_alignment/heading_error` | `std_msgs/Float32` | Ego heading minus nearest centerline heading, rad |

If the MGeo overlay is consistently shifted or rotated against MORAI, tune the transform offsets without editing the map:

```bash
roslaunch ajou_control mgeo_alignment_check.launch \
  offset_x:=0.0 offset_y:=560.0 offset_yaw_deg:=0.0 enable_rviz:=true
```

The launch file shows all MGeo link centerlines by default so the RViz view can be compared directly with the MORAI road layout. If RViz becomes slow, switch to ego-nearby rendering:

```bash
roslaunch ajou_control mgeo_alignment_check.launch \
  show_all_links:=false local_radius:=80.0 enable_rviz:=true
```

With only `link_set.json`, lane edges are estimated from `width_start` and `width_end`; they are useful for visual map matching but are not a replacement for the original `lane_boundary_set.json`. To hide the estimated edge lines:

```bash
roslaunch ajou_control mgeo_alignment_check.launch \
  show_estimated_lane_edges:=false enable_rviz:=true
```

Use a few fixed MORAI poses on lane centers and check that:

- ego marker is on the same road location in RViz and MORAI
- blue MGeo centerlines overlap MORAI lane centers
- white ego heading and green centerline heading point in the expected direction
- nearest distance is small when the vehicle is placed on a lane center

Only start MGeo trajectory generation after this alignment check is stable.

`mgeo_trajectory_generator.py` reads MGeo `link_set.json`, selects a stable route from the current MORAI ego pose by default, resamples link centerline `points`, and publishes two paths:

| Topic | Frame use | Meaning |
| --- | --- | --- |
| `/recorded_trajectory_path` | MORAI control frame | Path consumed by the controller and validator |
| `/mgeo_trajectory/preview_path` | RViz/MGeo preview frame | Same route transformed for RViz inspection |

```bash
roslaunch ajou_control mgeo_path_stack.launch
```

The default route generation waits briefly for `/Ego_topic`, chooses the nearest heading-compatible MGeo link, trims the path to start near the current ego position, then follows successor links with the smallest heading discontinuity for about `route_length_m`.

For control, the stack keeps `/Ego_topic.position` unmodified and instead converts MGeo points into MORAI coordinates for `/recorded_trajectory_path`:

```text
trajectory_x = mgeo_x + 134.24
trajectory_y = mgeo_y + 1507.36
trajectory_z = mgeo_z - 26.12
```

This is derived from the RViz alignment relation:

```text
MGeo-to-MORAI offset = map_offset - ego_offset
```

For RViz inspection, the stack separately publishes `/mgeo_trajectory/preview_path` using the same preview offsets as `mgeo_alignment_check.launch`, so the generated route remains visible near the aligned map instead of being shifted far into MORAI raw coordinates.

Default outputs:

| Topic | Type | Description |
| --- | --- | --- |
| `/recorded_trajectory_path` | `nav_msgs/Path` | MGeo centerline trajectory |
| `/mgeo_trajectory/preview_path` | `nav_msgs/Path` | RViz-friendly preview of the same generated route |
| `/recorded_trajectory/target_speed` | `std_msgs/Float32` | Target speed for the controller |
| `/mgeo_trajectory/markers` | `visualization_msgs/MarkerArray` | Trajectory markers |
| `/mgeo_trajectory/line` | `visualization_msgs/Marker` | Trajectory line strip |
| `/mgeo_trajectory/ego_comparison_markers` | `visualization_msgs/MarkerArray` | Ego pose, heading, nearest path point, and cross-track error |

Route selection options:

```bash
roslaunch ajou_control mgeo_path_stack.launch route_length_m:=180.0
roslaunch ajou_control mgeo_path_stack.launch start_link_id:=A2256W000532 max_links:=20
roslaunch ajou_control mgeo_path_stack.launch start_link_id:=A2256W000532 goal_link_id:=A2256W000600
roslaunch ajou_control mgeo_path_stack.launch route_link_ids:=A2256W000532,A2256W000533,A2256W000534
```

To visualize in RViz:

```bash
roslaunch ajou_control mgeo_path_stack.launch enable_rviz:=true
```

In RViz, set `Fixed Frame` to `map`, then add:

- `Path`: `/recorded_trajectory_path`
- `MarkerArray`: `/mgeo_trajectory/markers`
- `Marker`: `/mgeo_trajectory/line`
- `MarkerArray`: `/mgeo_trajectory/ego_comparison_markers`

To let the existing Pure Pursuit + PI controller follow the MGeo trajectory:

```bash
roslaunch ajou_control mgeo_path_stack.launch enable_controller:=true enable_validator:=true target_speed_mps:=3.0
```

The stack uses the MGeo-to-MORAI transform derived from `mgeo_alignment_check.launch`, so controller tracking and validation metrics are computed against raw MORAI ego coordinates.
