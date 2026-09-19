"""Class-conditioned motion policy for standardized tracked objects."""

from __future__ import annotations

import copy

from autoware_perception_msgs.msg import ObjectClassification

from .kinematic_prediction import dominant_label


def _zero_vector3(vector) -> None:
    vector.x = 0.0
    vector.y = 0.0
    vector.z = 0.0


def apply_object_motion_policy(message, force_unknown_stationary=True):
    """Return a copy with UNKNOWN objects constrained to zero motion.

    Pose, shape, identity, classification, confidence and covariance are
    preserved. Position observations may still update from frame to frame,
    but downstream prediction must not interpret their jitter as motion.

    Returns ``(output, constrained_count)``.
    """

    # A full deepcopy costs several milliseconds when a scene contains many
    # tracks. Share read-only non-UNKNOWN objects and copy only the objects
    # whose kinematics this policy changes.
    output = copy.copy(message)
    output.objects = list(message.objects)
    if not force_unknown_stationary:
        return output, 0

    constrained_count = 0
    for index, source in enumerate(message.objects):
        if dominant_label(source) != ObjectClassification.UNKNOWN:
            continue

        tracked = copy.deepcopy(source)
        output.objects[index] = tracked
        kinematics = tracked.kinematics
        twist = kinematics.twist_with_covariance.twist
        _zero_vector3(twist.linear)
        _zero_vector3(twist.angular)

        acceleration = kinematics.acceleration_with_covariance.accel
        _zero_vector3(acceleration.linear)
        _zero_vector3(acceleration.angular)
        kinematics.is_stationary = True
        constrained_count += 1

    return output, constrained_count
