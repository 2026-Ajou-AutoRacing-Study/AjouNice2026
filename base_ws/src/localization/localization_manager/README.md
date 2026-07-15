# localization_manager

This package connects FAST-LIO local odometry to the project's global TF tree
and publishes the vehicle-frame state expected by Planning.

## Interfaces

- Input `/localization/odom`: `camera_init -> body`, produced by FAST-LIO.
- Input `/Ego_topic`: used only for the first global alignment.
- Output TF: `map -> camera_init`.
- Output `/localization/kinematic_state`: `map -> base_link` pose and
  `base_link`-frame twist.

`fast_lio_tf.launch` also publishes the temporary hard-coded
`body -> base_link` static transform for the Sensor_1/2 IMU installation
(`[-1.528, 0.0, -0.200]`, identity rotation).

## TF ownership

Use `bringup/localization_fast_lio.launch` for the FAST-LIO tree. Do not run
`bringup/maptf_morai.launch` at the same time: that launch publishes
`morai_map -> base_link`, which would give `base_link` a second parent.

The global alignment source and topic/frame names are configured in
`configs/localization/localization_manager.yaml`.

## Current limitations

- Only `initialization.source: ego` is implemented.
- The initial Ego alignment is frozen; GNSS global correction is not yet
  implemented.
- Pose and twist covariance in `/localization/kinematic_state` are unavailable
  and remain zero.
- `body -> base_link` is hard-coded and must later be read from the selected
  sensor configuration.
