# Base4 Temporal 3-Sweep Handoff

## Scope

This branch implements the offline training path for
BEVFusion Base4 Temporal 3-Sweep.

Online ROS inference, sweep buffering, and LiDAR deskew are not
implemented here yet.

## LiDAR input contract

Each point has 5 features:

[x, y, z, intensity, time_lag_sec]

Three causal LiDAR sweeps are used:

t-2, t-1, t

The current sweep has:

time_lag_sec = 0

Past sweeps have negative time lag values.

Before concatenation, all past LiDAR points must be transformed
into the current LiDAR frame using ego-motion compensation.

Camera input remains current-frame only.

## Detection target

3D boxes use:

[x, y, z, length, width, height, yaw, vx, vy]

vx and vy are object velocity expressed in the current LiDAR
coordinate frame.

## Implemented

- 3-sweep temporal dataset builder
- ego-motion compensation
- Nx5 temporal LiDAR input
- 9D detection target including vx, vy
- MoraiBase4Temporal3SweepDataset
- TransFusion velocity head
- 4-channel -> 5-channel checkpoint adaptation
- temporal dataset validator
- temporal quality audit

## Validation completed

The following path was smoke-tested end-to-end:

rosbag
-> 3-sweep construction
-> ego-motion compensation
-> Nx5 temporal points
-> 9D velocity GT
-> MMDetection3D dataset
-> DataLoader
-> BEVFusion
-> TransFusion bbox loss
-> non-zero gradients in velocity head

## Checkpoint adaptation

The previous Base3 checkpoint uses 4 LiDAR input features.

The adaptation script expands the sparse-convolution input weight
from 4 channels to 5 channels.

The original four channels are copied unchanged and the new
time-lag channel is initialized to zero.

The velocity head is new and is therefore not present in the
previous checkpoint.

## Not implemented yet

- online three-sweep buffer
- online ego-pose alignment
- online past-to-current sweep transform
- LiDAR motion deskew
- online Nx5 point construction
- ROS inference integration

## Smoke-data warning

The temporary roundabout smoke dataset is diagnostic only.

An old MORAI bag exposed map-link-like actor names instead of
vehicle asset names, so a temporary K5 catalog alias was used only
under /tmp for pipeline validation.

Do not use that smoke dataset or alias catalog for training.
