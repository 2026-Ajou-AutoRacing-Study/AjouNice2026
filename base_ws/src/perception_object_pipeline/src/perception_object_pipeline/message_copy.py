"""Independent copies for the fixed ROS object-message schema.

These messages contain primitive covariance arrays and fixed geometry fields;
generic deepcopy unnecessarily traverses every float with a memo dictionary.
No mutable ROS submessage/list is shared by these helpers. The object ID and
header sharing already present in predictor conversion are handled separately.
"""
from __future__ import annotations

import copy


def copy_vector3(source):
    return type(source)(x=source.x, y=source.y, z=source.z)


def copy_pose(source):
    return type(source)(
        position=copy_vector3(source.position),
        orientation=type(source.orientation)(
            x=source.orientation.x, y=source.orientation.y,
            z=source.orientation.z, w=source.orientation.w),
    )


def copy_pose_covariance(source):
    return type(source)(pose=copy_pose(source.pose),
                        covariance=copy.copy(source.covariance))


def copy_twist_covariance(source):
    twist = source.twist
    return type(source)(
        twist=type(twist)(linear=copy_vector3(twist.linear),
                          angular=copy_vector3(twist.angular)),
        covariance=copy.copy(source.covariance),
    )


def copy_accel_covariance(source):
    accel = source.accel
    return type(source)(
        accel=type(accel)(linear=copy_vector3(accel.linear),
                          angular=copy_vector3(accel.angular)),
        covariance=copy.copy(source.covariance),
    )


def copy_shape(source):
    footprint = type(source.footprint)(
        points=[copy_vector3(point) for point in source.footprint.points])
    return type(source)(type=source.type, footprint=footprint,
                        dimensions=copy_vector3(source.dimensions))


def copy_classification(source):
    return [type(item)(label=item.label, probability=item.probability) for item in source]


def copy_tracked_object(source):
    output = copy.copy(source)
    output.object_id = copy.copy(source.object_id)
    output.object_id.uuid = copy.copy(source.object_id.uuid)
    output.classification = copy_classification(source.classification)
    output.shape = copy_shape(source.shape)
    output.kinematics = copy.copy(source.kinematics)
    output.kinematics.pose_with_covariance = copy_pose_covariance(
        source.kinematics.pose_with_covariance)
    output.kinematics.twist_with_covariance = copy_twist_covariance(
        source.kinematics.twist_with_covariance)
    output.kinematics.acceleration_with_covariance = copy_accel_covariance(
        source.kinematics.acceleration_with_covariance)
    return output
