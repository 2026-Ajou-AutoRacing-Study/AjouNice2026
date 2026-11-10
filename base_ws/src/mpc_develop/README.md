# mpc_develop

`mpc_develop` is an isolated data-collection package for nonlinear bicycle-model
identification.  It deliberately does not launch planning, localization,
perception, or an existing controller.

The package contains three operational scripts:

- `manual_command_publisher.py`: terminal-operated, bounded, type-1
  `morai_msgs/CtrlCmd` publisher.
- `automatic_excitation_publisher.py`: bounded fixed-speed, multisine-steering
  identification run. It requires explicit `--execute` and owns `/ctrl_cmd`
  exclusively for the duration of the run.
- `record_identification_bag.py`: explicit-topic `rosbag` recorder with a
  per-run manifest. It never uses `rosbag record -a`.

`bag_to_lateral_id_csv.py` converts a ROS 1 bag to a uniformly sampled CSV for
the MATLAB lateral model. It uses actual wheel angle from `/Ego_topic`, body
velocity from `/Ego_topic`, and yaw-rate/lateral-acceleration from `/sensor/imu`.
Raw command signals remain diagnostic columns; they are not substituted for
the measured wheel angle.

The MATLAB `idnlgrey` lateral dynamic-bicycle/Pacejka-E=0 model is in
[`matlab/`](matlab/README.md). It uses measured wheel angle and longitudinal
speed as inputs, and estimates only the six front/rear tire `B`, `C`, and `D`
parameters.

## Build

```bash
cd /home/kdy/AjouNice2026
catkin_make_isolated --install --pkg mpc_develop \
  --source "$PWD/base_ws/src" --build "$PWD/base_ws/build_aw149" \
  --devel "$PWD/base_ws/devel_aw149" --install-space "$PWD/base_ws/install_aw149" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j2 -l2
```

## Minimal MORAI collection session

This workstation has ROS 2 on the host; run this ROS 1 session in the supplied
Noetic container. Use four host terminals. Do not run an existing planner or
trajectory follower; the command publisher requires exclusive ownership of
`/ctrl_cmd`.

Terminal 1 creates the shared ROS 1 container and starts ROS master. Keep this
terminal open:

```bash
docker run --rm -it --name mpc-id-ros1 --network host \
  -v /home/kdy/AjouNice2026:/home/ajou/AjouNice2026 \
  -w /home/ajou/AjouNice2026 --entrypoint bash \
  ajounice2026-ros1:unified

source scripts/workspace/source_aw149.sh
# ROS Noetic Python packages (including rospkg) use the system interpreter.
export PATH=/usr/bin:$PATH
roscore
```

Terminal 2 enters that container and starts only the MORAI UDP bridge. This
supplies raw Ego, GPS, and IMU data and forwards `/ctrl_cmd` to MORAI; LiDAR is
intentionally disabled.

```bash
docker exec -it mpc-id-ros1 bash
cd /home/ajou/AjouNice2026
source scripts/workspace/source_aw149.sh
export PATH=/usr/bin:$PATH
roslaunch sensor_interface morai_udp_interface.launch \
  enable_lidar:=false \
  sensor_config:=/home/ajou/AjouNice2026/configs/sensors/Sensor_competition_local.json \
  network_config:=/home/ajou/AjouNice2026/configs/network/ego_network_competition_local.json
```

Terminal 3 records a labeled run.  It waits for `/Ego_topic`, `/sensor/imu`,
and `/sensor/gps` before starting.  It also records `/competition/raw_twist`,
which is the preferred source of body-frame velocity and yaw-rate samples for
lateral system identification.

```bash
docker exec -it mpc-id-ros1 bash
cd /home/ajou/AjouNice2026
source scripts/workspace/source_aw149.sh
export PATH=/usr/bin:$PATH
rosrun mpc_develop record_identification_bag.py \
  --label steering_step_v10_run01
```

In another terminal, run the command publisher.  It starts **disarmed** and
publishes no driving command until `arm` is entered.

```bash
docker exec -it mpc-id-ros1 bash
cd /home/ajou/AjouNice2026
source scripts/workspace/source_aw149.sh
export PATH=/usr/bin:$PATH
rosrun mpc_develop manual_command_publisher.py
```

Available operator commands:

```text
arm
set <accel_0to1> <brake_0to1> <physical_front_wheel_angle_rad>
zero
hold
stop
quit
```

For example, `set 0.15 0.0 0.05` requests 0.15 accelerator pedal and a
physical front-wheel angle of `+0.05 rad`.  Sending both throttle and brake is
prevented. `arm` begins with the configured brake hold; `stop` and `quit` send
one final brake-hold command before disarming.

Stop the recorder with `Ctrl-C`.  A finalized `identification.bag` and
`manifest.json` are written under:

```text
planner_logs/analysis/mpc_develop/<timestamp>_<label>/
```

For unattended simulation collection, start the recorder first, then run the
automatic excitation publisher in Terminal 4 instead of the manual publisher:

```bash
docker exec -it mpc-id-ros1 bash
cd /home/ajou/AjouNice2026
source scripts/workspace/source_aw149.sh
export PATH=/usr/bin:$PATH
rosrun mpc_develop automatic_excitation_publisher.py --execute
```

Its default run holds the brake, reaches 8 m/s, settles, applies a 60-second
bounded multisine steering signal (maximum 0.05 rad), then brake-holds and
exits. It aborts on missing Ego feedback, loss of the MORAI `/ctrl_cmd`
subscriber, a speed above 12 m/s, excitation speed below 3 m/s for 3 seconds,
or an existing `/ctrl_cmd` publisher.

## Convert a bag for MATLAB

Run conversion inside the Noetic container. The CSV columns are:

```text
t_s,segment_id,delta_rad,vx_mps,vy_mps,yaw_rate_radps,ay_mps2,
ctrl_steering_rad,ctrl_accel,ctrl_brake,heading_rad
```

```bash
rosrun mpc_develop bag_to_lateral_id_csv.py \
  --bag /path/to/input.bag \
  --output /path/to/lateral_id.csv
```

Defaults are 20 Hz, `speed >= 3 m/s`, actual wheel angle in degrees, and
body-frame `/Ego_topic.velocity`. The converter writes a neighboring
`*.metadata.json` summary, rejects interpolation across gaps longer than
0.15 seconds, and increments `segment_id` at each retained discontinuity.
Do not change measurement signs until plotting a left/right turn confirms the
MORAI sign convention. For a bag whose Ego velocity is known to be map-frame,
pass `--ego-velocity-frame map` and the appropriate `--heading-yaw-offset-rad`.

## Recorded topics

The recorder requests `/ctrl_cmd`, `/Ego_topic`, `/Competition_topic`,
`/competition/raw_twist`, raw IMU and GPS, collision data, `/tf`, `/tf_static`,
and command-status text.  `/Ego_topic` is a MORAI identification oracle; it
must not become a runtime MPC input.
