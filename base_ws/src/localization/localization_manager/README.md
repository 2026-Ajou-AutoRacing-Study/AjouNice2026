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

## FAST-LIO provider

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

## MORAI ground-truth provider

Run:

```bash
roslaunch bringup localization_morai_gt.launch
```

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
common Odometry output while applying the calibrated `-1.37 deg` yaw offset to
`morai_map -> base_link`. The raw `/Ego_topic` remains in `morai_map`.

## FAST-LIO + GNSS provider (experimental)

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

## GNSS + IMU provider (experimental baseline)

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
covariance, to `/localization/kinematic_state`. The copying node emits no TF.
This lets Planning keep the same input contract while the localization backend
changes.

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
- `configs/localization/gnss_imu_noise_sensor5_ideal.yaml`

The follow-up noise experiment is intentionally a separate pairing:

- `configs/sensors/Sensor_4_noise_v1.json`
- `configs/localization/gnss_imu_noise_sensor4_noise_v1.yaml`

The second covariance profile is an initial measurement model and must be
re-estimated from repeated noisy runs rather than treated as final calibration.
No gyro-bias/zeroing algorithm is enabled because the current MORAI setup has
zero-mean noise and no configured fixed bias.

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
- Pose and twist covariance in `/localization/kinematic_state` are unavailable.
- `body -> base_link` is hard-coded and must later be read from the selected
  sensor configuration.
- GNSS+IMU-only accuracy and outage behavior are not expected to match
  LiDAR+IMU+GNSS in geometrically rich environments; it is a lightweight
  baseline and fallback candidate.
