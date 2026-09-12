# MATP rawgt_v2 Epoch 80 release

## Model

Motion-Aware Temporal PointPainting (MATP)

- 3 temporal sweeps
- ego-motion compensated historical sweeps
- 10 point features
- motion-aware BEV temporal fusion
- 3D object detection
- BEV velocity regression (vx, vy)

## OpenPCDet config

pointpillar_morai_pointpainting_base2_B0_3sweep_10feat_velocity_range30_matp_motion_rawgt_v2.yaml

## Checkpoint

Expected filename:

checkpoint_epoch_80.pth

Training:

- epochs: 80
- seed: 666
- dataset: Base2 rawgt_v2
- classes:
  - Pedestrian
  - Vehicle
  - Obstacle

SHA256:

87801dd1019327e8a4d7519d9392fd33b866cb3f24267b8370df731591b7ddeb

The checkpoint binary is not stored directly in this Git repository.

Verify after download:

sha256sum checkpoint_epoch_80.pth

## Point feature order

x
y
z
intensity
background
pedestrian
vehicle
obstacle
unknown
time_lag_sec

The current sweep has time_lag_sec = 0.
Historical sweeps retain their measured negative time lag after ego-motion alignment.
