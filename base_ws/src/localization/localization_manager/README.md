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

## TF ownership and exclusivity

Do not run the FAST-LIO and MORAI GT localization launch files together. Do
not combine either one with `maptf.launch` or
`maptf_morai_deg_correction.launch`; those launch files publish competing map
or `base_link` TF edges.

The fixed `map -> morai_map` alignment is provided by
`tf_pub/map_tf.launch`. Provider topic/frame names and FAST-LIO initialization
are configured in `configs/localization/localization_manager.yaml`.

## Current limitations

- Only FAST-LIO `initialization.source: ego` is implemented.
- The initial FAST-LIO alignment is frozen; GNSS global correction is not yet
  implemented.
- Pose and twist covariance in `/localization/kinematic_state` are unavailable.
- `body -> base_link` is hard-coded and must later be read from the selected
  sensor configuration.
