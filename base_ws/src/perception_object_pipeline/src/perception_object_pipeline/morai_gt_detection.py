"""Convert MORAI object ground truth into the canonical detection contract."""

from __future__ import annotations

import copy
import math

from autoware_perception_msgs.msg import DetectedObjects

from .detected_objects import make_detected_objects


# MORAI vehicle ObjectStatus positions are actor reference points, not 3D box
# centers.  These values are the bbox-center-calibrated-v1 profiles documented
# by tracking_prediction_evaluation and keep GT boxes comparable to detector
# boxes before they enter the common tracker.
PASSENGER_VEHICLE_MAXIMUM_LENGTH_M = 5.5
PASSENGER_VEHICLE_LONGITUDINAL_OFFSET_M = 1.34
EXPLICIT_VEHICLE_REFERENCE_PROFILES = (
    {
        "length_m": 6.557,
        "width_m": 2.517,
        "height_m": 2.505,
        "longitudinal_offset_m": 1.383,
        "dimension_tolerance_m": 0.08,
    },
)


def vehicle_bbox_center_offset(
    length: float,
    width: float,
    height: float,
    *,
    passenger_maximum_length_m: float = PASSENGER_VEHICLE_MAXIMUM_LENGTH_M,
    passenger_longitudinal_offset_m: float = (
        PASSENGER_VEHICLE_LONGITUDINAL_OFFSET_M
    ),
) -> float:
    """Return the audited actor-reference to box-center offset, in metres."""
    if 0.0 < length <= passenger_maximum_length_m:
        return passenger_longitudinal_offset_m

    for profile in EXPLICIT_VEHICLE_REFERENCE_PROFILES:
        tolerance = profile["dimension_tolerance_m"]
        if all(
            abs(actual - profile[key]) <= tolerance
            for actual, key in (
                (length, "length_m"),
                (width, "width_m"),
                (height, "height_m"),
            )
        ):
            return profile["longitudinal_offset_m"]
    return 0.0


def _status_box(
    status,
    class_name: str,
    *,
    correct_vehicle_bbox_center: bool,
    passenger_maximum_length_m: float,
    passenger_longitudinal_offset_m: float,
):
    yaw = math.radians(float(status.heading))
    length = max(0.0, float(status.size.x))
    width = max(0.0, float(status.size.y))
    height = max(0.0, float(status.size.z))
    x = float(status.position.x)
    y = float(status.position.y)
    z = float(status.position.z)

    if class_name == "vehicle" and correct_vehicle_bbox_center:
        offset = vehicle_bbox_center_offset(
            length,
            width,
            height,
            passenger_maximum_length_m=passenger_maximum_length_m,
            passenger_longitudinal_offset_m=passenger_longitudinal_offset_m,
        )
        x += offset * math.cos(yaw)
        y += offset * math.sin(yaw)

    return [x, y, z, length, width, height, yaw]


def make_morai_gt_detections(
    message,
    *,
    frame_id: str = "morai_map",
    correct_vehicle_bbox_center: bool = True,
    passenger_maximum_length_m: float = PASSENGER_VEHICLE_MAXIMUM_LENGTH_M,
    passenger_longitudinal_offset_m: float = (
        PASSENGER_VEHICLE_LONGITUDINAL_OFFSET_M
    ),
) -> DetectedObjects:
    """Map ObjectStatusList poses/classes to detector-like boxes.

    Velocity is deliberately omitted.  This adapter replaces only object
    detection; the downstream EKF still estimates motion from detections.
    """
    header = copy.copy(message.header)
    header.frame_id = frame_id
    boxes = []
    class_names = []
    for class_name, statuses in (
        ("vehicle", message.npc_list),
        ("pedestrian", message.pedestrian_list),
        ("obstacle", message.obstacle_list),
    ):
        for status in statuses:
            boxes.append(
                _status_box(
                    status,
                    class_name,
                    correct_vehicle_bbox_center=correct_vehicle_bbox_center,
                    passenger_maximum_length_m=passenger_maximum_length_m,
                    passenger_longitudinal_offset_m=(
                        passenger_longitudinal_offset_m
                    ),
                )
            )
            class_names.append(class_name)

    return make_detected_objects(
        header,
        boxes,
        [1.0] * len(boxes),
        class_names,
        include_velocity=False,
    )
