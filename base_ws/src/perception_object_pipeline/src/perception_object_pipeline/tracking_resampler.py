"""Causal fixed-rate resampling for standardized tracked objects."""

from __future__ import annotations

import copy

from .kinematic_prediction import predict_pose


def extrapolate_tracked_objects(message, target_stamp):
    """Return a copy propagated from ``message.header.stamp`` to target time."""
    dt = target_stamp.to_sec() - message.header.stamp.to_sec()
    if dt < -1.0e-9:
        raise ValueError("target timestamp precedes tracked-object measurement")

    output = copy.deepcopy(message)
    output.header.stamp = target_stamp
    if dt <= 1.0e-9:
        return output

    for tracked in output.objects:
        tracked.kinematics.pose_with_covariance.pose = predict_pose(tracked, dt)
    return output
