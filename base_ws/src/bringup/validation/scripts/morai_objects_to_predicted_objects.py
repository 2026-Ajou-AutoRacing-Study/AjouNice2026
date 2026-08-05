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
    PredictedPath,
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


def stable_object_uuid(category, unique_id):
    """Generate a stable 128-bit ID without assuming MORAI IDs fit UUID fields."""
    value = uuid.uuid5(
        uuid.NAMESPACE_URL, "morai:{}:{}".format(category, unique_id)
    )
    return list(value.bytes)


def append_constant_velocity_prediction(
    output, horizon_sec, time_step_sec
):
    """Add the prediction trajectory expected by Autoware planning modules."""
    horizon_sec = max(0.0, float(horizon_sec))
    time_step_sec = max(0.05, float(time_step_sec))
    if horizon_sec <= 0.0:
        return

    pose = output.kinematics.initial_pose_with_covariance.pose
    velocity = output.kinematics.initial_twist_with_covariance.twist.linear
    speed = math.hypot(float(velocity.x), float(velocity.y))

    prediction = PredictedPath()
    prediction.confidence = 1.0
    prediction.time_step = rospy.Duration(time_step_sec)
    count = int(math.ceil(horizon_sec / time_step_sec))
    for step in range(count + 1):
        elapsed = min(step * time_step_sec, horizon_sec)
        item = copy.deepcopy(pose)
        item.position.x += float(velocity.x) * elapsed
        item.position.y += float(velocity.y) * elapsed
        item.position.z += float(velocity.z) * elapsed
        # A trajectory pose represents its direction of travel.  This is
        # especially important for a MORAI object whose body orientation and
        # local velocity were transformed into the map frame above.
        if speed > 1.0e-3:
            yaw = math.atan2(float(velocity.y), float(velocity.x))
            item.orientation.x = 0.0
            item.orientation.y = 0.0
            item.orientation.z = math.sin(yaw * 0.5)
            item.orientation.w = math.cos(yaw * 0.5)
        prediction.path.append(item)
    output.kinematics.predicted_paths = [prediction]


def make_predicted_object(
    detected_object,
    category,
    source_header,
    transform=None,
    position_is_bottom=True,
    velocity_scale_to_mps=1.0,
    acceleration_scale_to_mps2=1.0,
    classification_probability=1.0,
    prediction_horizon_sec=20.0,
    prediction_time_step_sec=0.5,
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
    # MORAI ObjectStatus velocity/acceleration are expressed in each object's
    # forward/lateral frame.  Convert them through the object's heading before
    # applying the source-frame -> map TF.  Applying only the TF rotation made
    # a vehicle heading south with ``velocity.x > 0`` look as if it were moving
    # east in the map frame, which prevented dynamic collision prediction.
    velocity = rotate_vector(
        detected_object.velocity,
        source_pose.pose.orientation,
        velocity_scale_to_mps,
    )
    acceleration = rotate_vector(
        detected_object.acceleration,
        source_pose.pose.orientation,
        acceleration_scale_to_mps2,
    )
    if rotation is not None:
        velocity = rotate_vector(velocity, rotation)
        acceleration = rotate_vector(acceleration, rotation)
    output.kinematics.initial_twist_with_covariance.twist.linear = velocity
    output.kinematics.initial_acceleration_with_covariance.accel.linear = (
        acceleration
    )
    append_constant_velocity_prediction(
        output, prediction_horizon_sec, prediction_time_step_sec
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
    prediction_horizon_sec=20.0,
    prediction_time_step_sec=0.5,
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
                    prediction_horizon_sec,
                    prediction_time_step_sec,
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
        self.prediction_horizon_sec = float(
            rospy.get_param("~prediction_horizon_sec", 20.0)
        )
        self.prediction_time_step_sec = float(
            rospy.get_param("~prediction_time_step_sec", 0.5)
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
                self.prediction_horizon_sec,
                self.prediction_time_step_sec,
            )
        )


def main():
    rospy.init_node("morai_objects_to_predicted_objects")
    MoraiObjectsToPredictedObjects()
    rospy.spin()


if __name__ == "__main__":
    main()
