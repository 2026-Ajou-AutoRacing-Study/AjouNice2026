# localization_manager

This package normalizes localization providers to the project's common TF and
Planning interfaces. Select exactly one provider through a top-level bringup
launch file.

## Common output contract

- TF lookup: `map -> base_link`
- Topic: `/localization/kinematic_state` (`nav_msgs/Odometry`)
- Pose: `map`-frame pose of `base_link`
- Twist: velocity of the `base_link` origin expressed in `base_link`

Provider-specific covariance handling is described below. FAST-LIO providers
currently leave it unavailable; the GNSS+IMU provider preserves the EKF
covariance.

## Production competition provider

The competition runtime uses GNSS+IMU+Competition GT CTRA-like localization.
Source `docs/common_alias.txt` and use exactly one of these entrypoints:

```bash
# Competition: /Competition_topic + GPS + IMU
localization

# MRP (2026 personal launcher) only: /Ego_topic + GPS + IMU
localization_mrp
```

`localization` applies the MR/competition `-1.347 deg` heading correction.
`localization_mrp` applies no correction because the audited MRP convention is
already aligned. This distinction follows the launcher convention, not ROS
versus UDP transport. Both aliases disable duplicate sensor static TF
publication. `/Ego_topic` direct GT localization is not a competition fallback.
See `docs/localization/README.md` for the canonical document index.

### Experimental smooth controller state

The production planning and TF contract remains
`/localization/kinematic_state`. For controlled A/B tests only, the provider can
also publish a controller-facing map-frame state that combines Local EKF motion
with a smoothed Global/Local alignment:

```bash
roslaunch localization_manager gnss_imu_gt_ctra.launch \
  publish_control_state:=true \
  control_translation_tau_sec:=0.5 \
  control_yaw_tau_sec:=0.5
```

The same arguments are passed through by
`bringup/localization_gnss_imu_gt_ctra.launch`, which should be used for full
provider replay and closed-loop tests.

The output is `/localization/control_kinematic_state` (`map -> base_link`
Odometry). The node publishes no TF and does not alter the global Planning
state. It is disabled by default and must not become the controller input until
closed-loop, GPS-outage/reacquisition, and checkpoint-reset gates pass.

### Experimental checkpoint-relocation guard

`localization_relocation_guard.py` compares co-timed GNSS and fused map poses.
It requires a 5 m innovation for three consecutive GNSS samples and applies a
five-second cooldown. The launch gate is disabled by default, and enabling it
still runs monitor-only unless explicitly overridden:

```bash
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  enable_relocation_guard:=true \
  relocation_guard_monitor_only:=true
```

Only an active experiment (`relocation_guard_monitor_only:=false`) publishes a
corrected-orientation pose to the Global EKF reset topic. The Local EKF is not
reset. Do not enable automatic reset in production until a real checkpoint
relocation bag and closed-loop recovery pass the documented gates.

## Legacy/research FAST-LIO provider

Run:

```bash
roslaunch bringup localization_fast_lio.launch
```

Inputs:

- `/localization/odom`: `camera_init -> body`, produced by FAST-LIO
- `/Ego_topic`: used only for the first global alignment

TF tree:

```text
map -> morai_map -> camera_init -> body -> base_link
```

`morai_map_camera_init_broadcaster.py` initializes and publishes
`morai_map -> camera_init`. FAST-LIO owns `camera_init -> body`.
`fast_lio.launch` also publishes the temporary hard-coded
`body -> base_link` static transform for the Sensor_1/2 IMU installation
(`[-1.528, 0.0, -0.200]`, identity rotation).

`fast_lio_kinematic_state_publisher.py` converts the body-origin twist to the
base-link origin using the lever arm, snapshots `map -> base_link`, and
publishes the common output topic.

## Legacy/local MORAI ground-truth provider

Run:

```bash
roslaunch bringup localization_morai_gt.launch
```

This provider requires `/Ego_topic`. It is retained for old bag reproduction
and local visualization, not competition operation.

Input:

- `/Ego_topic`

TF tree:

```text
map -> morai_map -> base_link
```

The existing `tf_pub/ego_tf.launch` owns `morai_map -> base_link`.
`morai_gt_kinematic_state_publisher.py` snapshots `map -> base_link` at the Ego
timestamp and copies MORAI's base-link-frame velocity into the common output
topic.

`bringup/maptf_morai.launch` remains only as a deprecated compatibility alias
for `bringup/localization_morai_gt.launch`.

`bringup/maptf_morai_deg_correction.launch` runs the same complete provider and
common Odometry output while applying the scenario-calibrated `-1.347 deg` yaw offset to
`morai_map -> base_link`. The raw `/Ego_topic` remains in `morai_map`.

## Legacy FAST-LIO + GNSS provider (experimental)

Run:

```bash
roslaunch bringup localization_fast_lio_gnss.launch
```

This provider keeps the same TF and Planning contracts as FAST-LIO. It adds:

- `morai_gps_to_navsatfix.py`: `/sensor/gps` to `/localization/gnss/fix`
- `/localization/initialpose`: latched starting base pose in `morai_map`
- `fast_lio_msf`: LiDAR/IMU estimation with GNSS position updates

The MSF's upstream 5 m displacement heading reset is disabled. Instead, the
initial Ego/manual pose determines the one-time MORAI ENU-to-`camera_init`
rotation, so GNSS correction does not rotate the existing TF tree mid-run.
Wheel input is disabled. The current GNSS covariance is a configurable test
model, not a calibrated competition value.

## Legacy GNSS + IMU provider (experimental baseline)

Run the noise-disabled Sensor_4 baseline:

```bash
roslaunch bringup localization_gnss_imu.launch
```

The provider deliberately keeps MORAI conversion outside
`robot_localization`:

```text
/sensor/imu (MORAI sensor_msgs/Imu with unknown covariance)
  -> imu_covariance_adapter.py
  -> /localization/imu (sensor_msgs/Imu with configured covariance)

/sensor/gps (morai_msgs/GPSMessage, WGS84)
  -> morai_gps_to_local_odometry.py
  -> /localization/gnss/odom (nav_msgs/Odometry, map-frame position)

first /Ego_topic pose (development) or configured manual competition pose
  -> gnss_imu_initial_pose_publisher.py
  -> /localization/initialpose
  -> global EKF set_pose
```

Two EKFs then own a non-conflicting REP-105 TF tree:

```text
local EKF:  odom -> base_link
global EKF: map -> odom
```

The global EKF output `/localization/odom` is copied, including pose and twist
covariance, to `/localization/kinematic_state`. The final-state publisher also
owns `map -> odom`, while the Local EKF independently owns
`odom -> base_link`. This lets Planning keep the same state and TF contract
while the localization backend changes.

The competition GT-assisted provider's final-state publisher additionally owns
`map -> odom`. It subscribes to Local EKF odometry and extrapolates that pose to
the Global EKF message timestamp before composing the transform. Do not replace
this with a latest (`Time(0)`) TF lookup: asynchronous 50 Hz EKF phases produced
up to tens of centimeters of TF closure error at speed.

GNSS+IMU uses the same `ego | manual` starting-pose policy as FAST-LIO+GNSS.
During development, the first valid `/Ego_topic` pose is transformed from
`morai_map` to `map` and resets only the global EKF. The local EKF keeps its
relative `odom` origin. For competition, set
`providers/gnss_imu/initialization/source: manual` and replace the configured
manual pose with the provided start pose. This initialization does not suppress
early `/localization/kinematic_state` messages; an output-readiness gate remains
a separate, deferred change.

The initial noise-disabled experiment uses:

- `configs/sensors/Sensor.json`
- `configs/localization/gnss_imu_noise_sensor6_ideal.yaml`

The follow-up noise experiment is intentionally a separate pairing:

- `configs/sensors/Sensor_4_noise_v1.json`
- `configs/localization/gnss_imu_noise_sensor4_noise_v1.yaml`

The second covariance profile is an initial measurement model and must be
re-estimated from repeated noisy runs rather than treated as final calibration.
No gyro-bias/zeroing algorithm is enabled because the current MORAI setup has
zero-mean noise and no configured fixed bias.

## GNSS localization bag + Lanelet RViz

The GNSS localization outputs are planar (`base_link.z = 0`), but the K-City
Lanelet OSM stores elevations around 28 m. The IONIQ5 mesh ground plane is also
0.3705 m below its rear-axle-centered `base_link`. Use the visualization-only
launch below instead of displaying the unmodified 3D Lanelet map:

```bash
roslaunch localization_manager gnss_imu_lanelet_rviz.launch \
  lanelet_map_file:=$HOME/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v67.osm \
  show_minimap:=true
```

The map path is explicit because `lanelet2_map_v67.osm` may be delivered
separately from the tracked repository. It uses the existing
`map_projector_info_v1.yaml`; no additional JSON metadata is required.

It performs these visualization-only operations:

1. flattens the Lanelet elevation;
2. publishes it in `lanelet_ground`, located 0.3705 m below `map`;
3. flattens and publishes the optional MGeo overlay at the same ground height;
4. restores the LiDAR/camera static TFs omitted from the compact result bag;
5. publishes a north-up global minimap from the Lanelet markers and localization
   state;
6. starts the IONIQ5 RobotModel and the localization evaluation RViz config.

The localization pose, TF, sensor configuration, OSM source, and vehicle URDF
are not modified. Override `lanelet_z_offset` only if the vehicle mesh or
base-link convention changes.

The compact localization-result bag does not contain LiDAR or camera messages.
Replay it together with the timestamp-matched raw bag and select only the
required topics:

```bash
rosbag play --clock --pause \
  rosbags/raw_sensor/competition_route_full_highspeed.bag \
  rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag \
  --topics \
  /sensor/lidar1 \
  /sensor/camera/front/compressed \
  /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed \
  /sensor/camera/back/compressed \
  /localization/kinematic_state \
  /localization/acceleration \
  /tf \
  /tf_static
```

The RViz Odometry display uses `/localization/kinematic_state`. Full GT and
aligned localization paths are independent displays on:

- `/localization_state_eval/ego_path`
- `/localization_state_eval/state_aligned_path`

Publish those latched paths from the evaluator when a full-run comparison is
needed. Pass `--rviz-ground-z -0.3105` when using the flattened Lanelet launch;
this matches the Lanelet centerline height and changes only the published RViz
geometry, not the evaluated values. The paths are not required for the moving
vehicle, LiDAR, or camera replay.

The integrated RViz launch publishes only the MGeo link centerline by default.
Width-derived edges, nodes, signals, origin axes, and metadata are disabled
because they are not needed for the localization comparison. In particular,
the width edges are inferred from each link's width rather than native
boundary geometry.

```bash
roslaunch localization_manager gnss_imu_lanelet_rviz.launch
```

The launch starts the localization state overlay by default. It subscribes to
`/localization/kinematic_state` and publishes
`/localization/debug/state_markers`:

- white text above `base_link`: horizontal speed, `vx`, `vy`, yaw rate,
  state arrival rate, and message age;
- cyan arrow: horizontal velocity direction and scaled magnitude.

Disable only this debug visualization with `show_state_overlay:=false`.
The optional blue `mgeo_inferred_width_edges` lines are not surveyed/native
road boundaries. They are inferred by offsetting each MGeo link centerline by
half of `width_start`/`width_end`, so they can differ from LiDAR-visible paint.

The same launch also starts `localization_minimap_publisher.py` by default.
It publishes `/localization/minimap/image` as a `480x480` `bgr8`
`sensor_msgs/Image` at 10 Hz. The `Global Minimap` RViz Image display shows the
north-up Lanelet overview, current localization pose and heading, accumulated
state path, and horizontal speed. The image is regenerated from the existing
map and state during replay, so it does not need to be recorded in the bag.
Disable it with `show_minimap:=false`.

## GNSS + IMU + GT CTRA-like provider (production)

Run production through the common alias:

```bash
localization
```

For existing bags or personal MORAI that contain `/Ego_topic` but not
`/Competition_topic`, use:

```bash
localization_mrp
```

The aliases expand to explicit source and heading arguments. When debugging
without aliases, the competition equivalent is:

```bash
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  gt_source:=competition \
  use_heading_correction:=true \
  heading_yaw_offset_deg:=-1.347 \
  publish_sensor_tf:=false
```

The provider adds `morai_gt_motion_adapter.py` without replacing the existing
GNSS or IMU adapters:

```text
/Ego_topic or /Competition_topic
  -> /localization/gt/orientation  (absolute yaw)
  -> /localization/gt/twist        (base-link longitudinal vx)

/sensor/imu
  -> /localization/imu             (only yaw rate and ax are fused)

/sensor/gps
  -> /localization/gnss/odom       (global x/y)
```

Both local and global EKFs fuse GT yaw, GT longitudinal velocity, IMU yaw rate,
and IMU longitudinal acceleration. The global EKF additionally fuses GNSS x/y.
IMU orientation, lateral acceleration, and lateral velocity are not fused.
`frequency: 50` and `predict_to_current_time: true` use the EKF process model
to publish a regular high-rate state between direct GT/GNSS updates.

The first `/Ego_topic` pose remains a separate development-only initialization
input. Selecting `gt_source:=competition` changes only the continuous yaw and
velocity source. Replace the initial Ego pose with the provided manual start
pose before competition use.

The current production contract and validation scope are documented in
`docs/localization/README.md` and
`docs/localization/260918_competition_localization_critical_issue_status.md`.
The initial implementation record is preserved under
`docs/localization/legacy/260801_robot_localization_gnss_imu_gt_ctra_fusion.md`.

An experimental controller-only output can be enabled with
`publish_control_state:=true`.  The default strategy low-pass filters the
Global/Local alignment.  Adding `use_quantized_gnss_control_state:=true`
instead treats each GNSS value as a quantized cell, applies only the minimum
lateral correction needed to remain in that cell, and preserves the Global
EKF longitudinal pose and yaw.  Both strategies are default-off and own no TF;
see `docs/localization/260925_QUANTIZED_GNSS_CONTROL_STATE_EVIDENCE.md` before
using either one in a controller A/B.
The quantized strategy falls back to the Global state when GNSS is stale,
waits one second after reacquisition, and ignores cell corrections whose
GNSS-to-Global innovation exceeds 0.5 m.
Its cell is rotated `-1.347 deg` in the map frame: three raw bags independently
showed the MORAI `1/600000 deg` WGS84 lattice projects to a UTM 52N east axis
at approximately `-1.348 deg` in K-City.

The separate checkpoint-relocation guard is disabled and monitor-only by
default. Its persistent-sample counter resets after a GNSS gap longer than
0.5 s or a non-monotonic timestamp, so samples on opposite sides of an outage
cannot satisfy one relocation sequence.

## TF ownership and exclusivity

Do not run any two provider launch files together. Do
not combine either one with `maptf.launch` or
`maptf_morai_deg_correction.launch`; those launch files publish competing map
or `base_link` TF edges.

The fixed `map -> morai_map` alignment is provided by
`tf_pub/map_tf.launch`. Provider topic/frame names and FAST-LIO initialization
are configured in `configs/localization/localization_manager.yaml`.

## Current limitations

- FAST-LIO, FAST-LIO+GNSS, and GNSS+IMU initialization support `ego` and
  `manual` sources. The GNSS+IMU `ego` path has been exercised with current
  MORAI bags; its `manual` path still requires competition-interface validation.
- FAST-LIO without GNSS still has a frozen initial global alignment.
- FAST-LIO+GNSS is experimental and must be validated against multiple noise,
  stop/go, turn, and GNSS-outage scenarios.
- GNSS+IMU+GT currently rejects duplicate GT timestamps but has not yet been
  validated against a native `/Competition_topic` recording.
- GNSS+IMU providers reject MORAI's zero-latitude/longitude outage sentinel
  before publishing `/localization/gnss/odom`. During the outage the EKF keeps
  predicting from its remaining motion inputs; GNSS recovery jump handling
  still requires scenario-level validation.
- Pose and twist covariance in `/localization/kinematic_state` are unavailable.
- `body -> base_link` is hard-coded and must later be read from the selected
  sensor configuration.
- GNSS+IMU-only accuracy and outage behavior are not expected to match
  LiDAR+IMU+GNSS in geometrically rich environments; it is a lightweight
  baseline and fallback candidate.
