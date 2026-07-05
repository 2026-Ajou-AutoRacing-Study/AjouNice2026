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

It compares `/Ego_topic` against `/recorded_trajectory_path` and reports:

| Metric | Meaning |
| --- | --- |
| `lateral` | Current signed lateral error in meters |
| `lateral_rms` | RMS lateral error over the validation run |
| `lateral_max` | Maximum absolute lateral error |
| `heading_rms` | RMS heading error in radians |
| `speed_rms` | RMS speed error in m/s |
| `progress` | Path completion ratio |
| `steer_sat` | Fraction of samples near steering saturation |

The validator also writes a per-sample CSV:

```text
~/catkin_ws/src/ajou_control/validation_tracking.csv
```

## Current Scope

Implemented:

- Virtual trajectory generator for path/topic validation.
- Recorded path tracking with Pure Pursuit steering.
- Longitudinal PI speed control publishing MORAI `CtrlCmd`.
- RViz debug markers for ego, nearest point, target point, lookahead, and cross-track error.
- Tracking validator with RMS lateral, heading, speed, progress, and steering saturation metrics.

The skeleton scripts are kept only as reference placeholders; use `pure_pursuit_pi_controller.py` and `tracking_validator.py` for active testing.
