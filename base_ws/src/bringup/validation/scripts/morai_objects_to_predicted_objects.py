#!/usr/bin/env python3
"""Bridge MORAI ObjectStatusList into Autoware PredictedObjects."""

import copy
import math
import uuid

import rospy
import tf2_geometry_msgs
import tf2_ros
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    Shape,
)
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from morai_msgs.msg import ObjectStatusList


CATEGORY_LABELS = {
    "npc": ObjectClassification.CAR,
    "pedestrian": ObjectClassification.PEDESTRIAN,
    "obstacle": ObjectClassification.UNKNOWN,
}


def bounded_dimension(value):
    """Keep malformed or zero-sized MORAI objects usable by the planner."""
    return max(0.10, abs(float(value)))


def rotate_vector(vector, quaternion, scale=1.0):
    """Rotate a vector by a geometry_msgs Quaternion."""
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        x = y = z = 0.0
        w = 1.0
    else:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    vx = float(vector.x) * float(scale)
    vy = float(vector.y) * float(scale)
    vz = float(vector.z) * float(scale)

    # Quaternion-derived rotation matrix.
    output = Vector3()
    output.x = (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz
    )
    output.y = (
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz
    )
    output.z = (
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz
    )
    return output


def rotate_vector_by_yaw(vector, yaw, scale=1.0):
    """Rotate an object-frame vector into its MORAI source frame."""
    vx = float(vector.x) * float(scale)
    vy = float(vector.y) * float(scale)

    output = Vector3()
    output.x = math.cos(yaw) * vx - math.sin(yaw) * vy
    output.y = math.sin(yaw) * vx + math.cos(yaw) * vy
    output.z = float(vector.z) * float(scale)
    return output


def stable_object_uuid(category, unique_id):
    """Generate a stable 128-bit ID without assuming MORAI IDs fit UUID fields."""
    value = uuid.uuid5(
        uuid.NAMESPACE_URL, "morai:{}:{}".format(category, unique_id)
    )
    return list(value.bytes)


def make_predicted_object(
    detected_object,
    category,
    source_header,
    transform=None,
    position_is_bottom=True,
    velocity_scale_to_mps=1.0,
    acceleration_scale_to_mps2=1.0,
    classification_probability=1.0,
    kinematics_in_object_frame=True,
):
    """Convert one MORAI object, optionally applying source-to-target TF."""
    size_x = bounded_dimension(detected_object.size.x)
    size_y = bounded_dimension(detected_object.size.y)
    size_z = bounded_dimension(detected_object.size.z)

    source_pose = PoseStamped()
    source_pose.header = copy.deepcopy(source_header)
    source_pose.pose.position.x = float(detected_object.position.x)
    source_pose.pose.position.y = float(detected_object.position.y)
    source_pose.pose.position.z = float(detected_object.position.z)
    if position_is_bottom:
        source_pose.pose.position.z += size_z * 0.5

    yaw = math.radians(float(detected_object.heading))
    source_pose.pose.orientation.z = math.sin(yaw * 0.5)
    source_pose.pose.orientation.w = math.cos(yaw * 0.5)

    if transform is None:
        target_pose = source_pose
        rotation = None
    else:
        target_pose = tf2_geometry_msgs.do_transform_pose(
            source_pose, transform
        )
        rotation = transform.transform.rotation

    output = PredictedObject()
    output.object_id.uuid = stable_object_uuid(
        category, detected_object.unique_id
    )
    output.existence_probability = 1.0

    classification = ObjectClassification()
    classification.label = CATEGORY_LABELS[category]
    classification.probability = max(
        0.0, min(1.0, float(classification_probability))
    )
    output.classification = [classification]

    output.kinematics.initial_pose_with_covariance.pose = target_pose.pose
    # MORAI ObjectPacket velocity/acceleration are expressed in each object's
    # heading frame.  Convert them to the MORAI map frame first, then apply
    # the static MORAI-map -> map transform.  Skipping the first rotation made
    # a pedestrian walking across the lane appear to move sideways in map,
    # so the dynamic-obstacle swept-path check did not intersect ego's path.
    if kinematics_in_object_frame:
        source_velocity = rotate_vector_by_yaw(
            detected_object.velocity, yaw, velocity_scale_to_mps
        )
        source_acceleration = rotate_vector_by_yaw(
            detected_object.acceleration, yaw, acceleration_scale_to_mps2
        )
    else:
        source_velocity = Vector3(
            x=float(detected_object.velocity.x) * velocity_scale_to_mps,
            y=float(detected_object.velocity.y) * velocity_scale_to_mps,
            z=float(detected_object.velocity.z) * velocity_scale_to_mps,
        )
        source_acceleration = Vector3(
            x=float(detected_object.acceleration.x) * acceleration_scale_to_mps2,
            y=float(detected_object.acceleration.y) * acceleration_scale_to_mps2,
            z=float(detected_object.acceleration.z) * acceleration_scale_to_mps2,
        )

    velocity = rotate_vector(
        source_velocity,
        rotation if rotation is not None else Quaternion(w=1.0),
    )
    acceleration = rotate_vector(
        source_acceleration,
        rotation if rotation is not None else Quaternion(w=1.0),
    )
    output.kinematics.initial_twist_with_covariance.twist.linear = velocity
    output.kinematics.initial_acceleration_with_covariance.accel.linear = (
        acceleration
    )

    output.shape.type = Shape.BOUNDING_BOX
    output.shape.dimensions.x = size_x
    output.shape.dimensions.y = size_y
    output.shape.dimensions.z = size_z
    return output


def make_predicted_objects(
    message,
    transform=None,
    target_frame="map",
    fallback_source_frame="morai_map",
    position_is_bottom=True,
    velocity_scale_to_mps=1.0,
    acceleration_scale_to_mps2=1.0,
    classification_probability=1.0,
    kinematics_in_object_frame=True,
):
    """Convert a complete MORAI object list into planner input."""
    source_header = copy.deepcopy(message.header)
    if not source_header.frame_id:
        source_header.frame_id = fallback_source_frame

    output = PredictedObjects()
    output.header = copy.deepcopy(source_header)
    output.header.frame_id = target_frame

    groups = (
        ("npc", message.npc_list),
        ("pedestrian", message.pedestrian_list),
        ("obstacle", message.obstacle_list),
    )
    for category, detected_objects in groups:
        for detected_object in detected_objects:
            output.objects.append(
                make_predicted_object(
                    detected_object,
                    category,
                    source_header,
                    transform,
                    position_is_bottom,
                    velocity_scale_to_mps,
                    acceleration_scale_to_mps2,
                    classification_probability,
                    kinematics_in_object_frame,
                )
            )
    return output


class MoraiObjectsToPredictedObjects:
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.fallback_source_frame = rospy.get_param(
            "~fallback_source_frame", "morai_map"
        )
        self.position_is_bottom = bool(
            rospy.get_param("~position_is_bottom", True)
        )
        self.velocity_scale_to_mps = float(
            rospy.get_param("~velocity_scale_to_mps", 1.0)
        )
        self.acceleration_scale_to_mps2 = float(
            rospy.get_param("~acceleration_scale_to_mps2", 1.0)
        )
        self.classification_probability = float(
            rospy.get_param("~classification_probability", 1.0)
        )
        self.kinematics_in_object_frame = bool(
            rospy.get_param("~kinematics_in_object_frame", True)
        )
        self.tf_timeout_sec = float(rospy.get_param("~tf_timeout_sec", 0.2))
        input_topic = rospy.get_param("~input_topic", "/Object_topic")
        output_topic = rospy.get_param(
            "~output_topic", "/perception/object_recognition/objects"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            output_topic, PredictedObjects, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            input_topic, ObjectStatusList, self.on_objects, queue_size=1
        )
        rospy.loginfo(
            "MORAI planner object bridge: %s -> %s (frame=%s)",
            input_topic,
            output_topic,
            self.target_frame,
        )

    def on_objects(self, message):
        source_frame = message.header.frame_id or self.fallback_source_frame
        transform = None
        if source_frame != self.target_frame:
            stamp = message.header.stamp
            if stamp == rospy.Time():
                stamp = rospy.Time(0)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    stamp,
                    rospy.Duration(self.tf_timeout_sec),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as error:
                rospy.logwarn_throttle(
                    2.0,
                    "MORAI object bridge waiting for TF %s <- %s: %s",
                    self.target_frame,
                    source_frame,
                    error,
                )
                return

        self.publisher.publish(
            make_predicted_objects(
                message,
                transform,
                self.target_frame,
                self.fallback_source_frame,
                self.position_is_bottom,
                self.velocity_scale_to_mps,
                self.acceleration_scale_to_mps2,
                self.classification_probability,
                self.kinematics_in_object_frame,
            )
        )


def main():
    rospy.init_node("morai_objects_to_predicted_objects")
    MoraiObjectsToPredictedObjects()
    rospy.spin()


if __name__ == "__main__":
    main()
