"""Build Autoware-compatible detection messages from model box tensors.

All positions and dimensions follow the detector output frame:
``x`` forward, ``y`` left and ``z`` up.  The tracker is responsible for
transforming this measurement into the world frame at ``header.stamp``.
"""

from __future__ import annotations

import copy
import math
from typing import Iterable, Mapping, Sequence

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    Shape,
)
from geometry_msgs.msg import Quaternion, Vector3


CLASS_LABELS = {
    "vehicle": ObjectClassification.CAR,
    "pedestrian": ObjectClassification.PEDESTRIAN,
    "obstacle": ObjectClassification.UNKNOWN,
}

ORIENTATION_AVAILABILITY = {
    "vehicle": DetectedObjectKinematics.AVAILABLE,
    # The current datasets do not define a reliable semantic front direction
    # for pedestrians and many obstacles.  Their box major axis is still
    # useful, but its sign must not be interpreted as facing direction.
    "pedestrian": DetectedObjectKinematics.SIGN_UNKNOWN,
    "obstacle": DetectedObjectKinematics.SIGN_UNKNOWN,
}


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    half = 0.5 * float(yaw)
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def make_detected_object(
    box: Sequence[float],
    score: float,
    class_name: str,
    *,
    class_labels: Mapping[str, int] = CLASS_LABELS,
    orientation_availability: Mapping[str, int] = ORIENTATION_AVAILABILITY,
    include_velocity: bool = False,
    velocity_classes: Sequence[str] = ("vehicle",),
    velocity_noise_std_mps: float = 1.0,
    maximum_velocity_mps: float = 30.0,
) -> DetectedObject:
    """Convert ``[x, y, z, length, width, height, yaw]`` to one object."""
    if len(box) < 7:
        raise ValueError("3D detection box must contain at least 7 values")

    probability = min(1.0, max(0.0, float(score)))
    output = DetectedObject()
    output.existence_probability = probability
    output.classification = [
        ObjectClassification(
            label=int(class_labels.get(class_name, ObjectClassification.UNKNOWN)),
            probability=probability,
        )
    ]

    output.kinematics.pose_with_covariance.pose.position.x = float(box[0])
    output.kinematics.pose_with_covariance.pose.position.y = float(box[1])
    output.kinematics.pose_with_covariance.pose.position.z = float(box[2])
    output.kinematics.pose_with_covariance.pose.orientation = _quaternion_from_yaw(box[6])
    output.kinematics.has_position_covariance = False
    output.kinematics.orientation_availability = int(
        orientation_availability.get(
            class_name, DetectedObjectKinematics.SIGN_UNKNOWN
        )
    )
    output.kinematics.has_twist = False
    output.kinematics.has_twist_covariance = False
    if include_velocity and len(box) >= 9 and class_name in velocity_classes:
        velocity_x = float(box[7])
        velocity_y = float(box[8])
        speed = math.hypot(velocity_x, velocity_y)
        if (
            math.isfinite(velocity_x)
            and math.isfinite(velocity_y)
            and speed <= max(0.0, float(maximum_velocity_mps))
        ):
            # The PointPillars target is a vector in detector/LiDAR axes.
            # Our standardized DetectedObject contract stores twist in the
            # object's body axes; the EKF rotates it back to map VX/VY.
            cosine = math.cos(float(box[6]))
            sine = math.sin(float(box[6]))
            twist = output.kinematics.twist_with_covariance.twist.linear
            twist.x = cosine * velocity_x + sine * velocity_y
            twist.y = -sine * velocity_x + cosine * velocity_y
            variance = max(0.0, float(velocity_noise_std_mps)) ** 2
            covariance = output.kinematics.twist_with_covariance.covariance
            covariance[0] = variance
            covariance[7] = variance
            output.kinematics.has_twist = True
            output.kinematics.has_twist_covariance = True

    output.shape.type = Shape.BOUNDING_BOX
    output.shape.dimensions = Vector3(
        x=max(0.0, float(box[3])),
        y=max(0.0, float(box[4])),
        z=max(0.0, float(box[5])),
    )
    return output


def make_detected_objects(
    header,
    boxes: Iterable[Sequence[float]],
    scores: Iterable[float],
    class_names: Iterable[str],
    *,
    include_velocity: bool = False,
    velocity_classes: Sequence[str] = ("vehicle",),
    velocity_noise_std_mps: float = 1.0,
    maximum_velocity_mps: float = 30.0,
) -> DetectedObjects:
    """Build one frame of standardized detector output."""
    output = DetectedObjects()
    output.header = copy.copy(header)
    output.objects = [
        make_detected_object(
            box,
            score,
            class_name,
            include_velocity=include_velocity,
            velocity_classes=velocity_classes,
            velocity_noise_std_mps=velocity_noise_std_mps,
            maximum_velocity_mps=maximum_velocity_mps,
        )
        for box, score, class_name in zip(boxes, scores, class_names)
    ]
    return output
