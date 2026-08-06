# localization_manager

This package normalizes localization providers to the project's common TF and
Planning interfaces. Select exactly one provider through a top-level bringup
launch file.

## Common output contract

- TF lookup: `map -> base_link`
- Topic: `/localization/kinematic_state` (`nav_msgs/Odometry`)
- Pose: `map`-frame pose of `base_link`
- Twist: velocity of the `base_link` origin expressed in `base_link`

Pose and twist covariance are currently unavailable and remain zero.

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

## TF ownership and exclusivity

Do not run any two provider launch files together. Do
not combine either one with `maptf.launch` or
`maptf_morai_deg_correction.launch`; those launch files publish competing map
or `base_link` TF edges.

The fixed `map -> morai_map` alignment is provided by
`tf_pub/map_tf.launch`. Provider topic/frame names and FAST-LIO initialization
are configured in `configs/localization/localization_manager.yaml`.

## Current limitations

- FAST-LIO and FAST-LIO+GNSS initialization support `ego` and `manual` sources;
  only the `ego` path has been exercised with the current MORAI bag.
- FAST-LIO without GNSS still has a frozen initial global alignment.
- FAST-LIO+GNSS is experimental and must be validated against multiple noise,
  stop/go, turn, and GNSS-outage scenarios.
- Pose and twist covariance in `/localization/kinematic_state` are unavailable.
- `body -> base_link` is hard-coded and must later be read from the selected
  sensor configuration.
