# MORAI Autoware Interface

ROS 2 Humble interface between MORAI `25.S4.MolitComp03` and the pinned
Autoware Universe `0.51.0` workspace.

Pinned source baseline:

- Autoware manifest: `4847819760656247343dfb995ea8e151fc3d9b00`
- `autoware_universe`: `0.51.0` / `d4d260983d357e1b2b34291d91933f9f4b53bf94`
- `autoware_core`: `1.8.0` / `16045061a9c10da468b60e190b8fab02110fa501`
- `autoware_launch`: `0.51.0` / `3ba12ba6a47abca105e755e6d21cf66a0cdf743f`

The package follows Autoware's vehicle-interface contract and the MORAI
simulator compatibility tests. It does not modify Autoware planning or control
algorithms.

This is an additive adapter package. It does not patch the source of
`autoware_universe`, `autoware_core`, or `autoware_launch`. The separate
`morai_rosbridge_compat.py` launcher adjusts rosbridge JSON decoding for the
competition build; it also does not modify Autoware algorithms.

## Interfaces

MORAI to Autoware:

- `/Ego_topic` -> velocity, steering and actuation status
- `/Ego_topic` -> `/localization/kinematic_state` and acceleration
- first valid `/Ego_topic` -> latched localization `INITIALIZED` state
- `/Ego_topic` -> normalized `map` to `base_link` transform
- `/Object_topic` -> `/perception/object_recognition/objects`
- Event service responses -> control mode, gear and lamp reports

Autoware to MORAI:

- `/control/command/control_cmd` -> `/ctrl_cmd`
- `/control/command/gear_cmd` -> `/Service_MoraiEventCmd`
- turn and hazard commands -> `/Service_MoraiEventCmd`
- `/control/control_mode_request` -> `/Service_MoraiEventCmd`

The default longitudinal interface is MORAI target-velocity mode. Incoming ego
velocity is passed through in m/s, incoming `wheel_angle` is converted from
degrees to radians, and outgoing Autoware velocity in m/s is converted to MORAI
km/h. Object-list velocity is converted from km/h to m/s. These defaults match
the measured `25.S4.MolitComp03` competition interface.

## Build

```bash
cd /home/ajou/autoware-0.51
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select morai_autoware_interface
source install/setup.bash
```

## Run

Start the already-created competition-compatible rosbridge first:

```bash
python3 /home/ajou/autoware-0.51/morai_rosbridge_compat.py \
  --ros-args -p port:=9090 -p max_message_size:=10000000
```

Then start this interface:

```bash
ros2 launch morai_autoware_interface morai_autoware_interface.launch.py
```

Safe startup is the default. The node reports manual/park and does not change
MORAI control mode until Autoware calls `/control/control_mode_request` and
publishes a gear command. Set
`initial.set_autonomous_and_drive_on_start: true` only for a controlled test.
Repeated gear and lamp commands that already match the reported state are
ignored instead of repeatedly calling the MORAI event service.

If an Autoware control command is not refreshed for 0.5 seconds while the
reported mode is autonomous, the interface publishes full brake.

## First checks

```bash
ros2 topic hz /vehicle/status/velocity_status
ros2 topic echo --once /localization/kinematic_state
ros2 topic echo --once /perception/object_recognition/objects
ros2 service type /control/control_mode_request
```

## One-command safe verification

Keep MORAI and the compatibility rosbridge connected, then run:

```bash
cd /home/ajou/autoware-0.51
./src/vehicle/external/morai_autoware_interface/scripts/verify_safe.sh
```

The verifier compares messages with identical timestamps, checks live rates
and topic/service types, converts a synthetic Autoware command, and verifies
the command-timeout brake watchdog. All command outputs are redirected to
`/verification/morai_ctrl_cmd`; the real MORAI `/ctrl_cmd` receives nothing.
The command exits with status `0` on PASS and `1` on FAIL.

Do not run `autoware_simple_planning_simulator` at the same time. MORAI is the
vehicle simulator in this configuration.

The competition bridge currently advertises `/clock` but does not publish clock
messages. Keep Autoware and this adapter on `use_sim_time:=false` unless MORAI
clock publication is enabled and verified.

## Parameters requiring track validation

- `command.steering_sign`: invert to `-1.0` if a positive Autoware command
  turns MORAI right.
- `report.steering_sign`: invert independently if MORAI steering reports use
  the opposite sign.
- `report.ego_velocity_is_kmh`: keep `false` for the measured competition
  build; legacy MORAI interfaces that publish km/h require `true`.
- `report.object_velocity_is_kmh`: MORAI object-list velocity is documented as
  km/h, so the competition default is `true`.
- `report.morai_steering_is_degrees`: keep `true` for the measured competition
  build.
- `command.max_velocity_kmh`: safety cap; set to the competition limit.
- `frames.map`: the K-City Lanelet2/point-cloud map must use the same origin as
  the MORAI scenario coordinates.

Object conversion is ground truth for planning/control integration. Disable
`publish.objects` when evaluating the original Autoware perception stack from
simulated lidar and camera sensors.

Each converted object includes a five-second constant-velocity predicted path
at 0.5-second intervals. Stationary MORAI objects therefore keep a stationary
path while moving objects are projected in their reported heading frame.

## Verified on the competition build

With the local competition simulator connected through the compatibility
rosbridge, the package has been checked for:

- live `/Ego_topic` conversion to velocity, steering, acceleration, odometry,
  and `map -> base_link` TF;
- live `/Object_topic` obstacle conversion to `PredictedObjects`;
- matching BEST_EFFORT QoS between the adapter `/ctrl_cmd` publisher and the
  rosbridge subscriber;
- dry-run control conversion with `/ctrl_cmd` remapped away from MORAI:
  `2.0 m/s`, `0.5 m/s^2`, and `0.1 rad` became `7.2 km/h`, `0.5 m/s^2`, and
  `0.1 rad`;
- clean package build, clean node shutdown, and all conversion tests passing.

This verifies the bridge, adapter, and a continuous 11.72 m closed-loop drive.
It does not verify the complete competition course. The K-City v60 phase-1
route was accepted and produced a trajectory at approximately 10 Hz, but the
physical two-phase course drive was not completed when this snapshot was made.
