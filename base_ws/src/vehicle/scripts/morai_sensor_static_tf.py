#!/usr/bin/env python3
import json
import math
import os

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


SENSOR_GROUPS = (
    ("Lidar3DList", "lc", "publish_lidar"),
    ("IMUList", "ic", "publish_imu"),
    ("cameraList", "cc", "publish_camera"),
    ("GPSList", "gc", "publish_gps"),
)


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def numeric_triplet(values, fields, unit_scale=1.0):
    return [float(values[field]) * unit_scale for field in fields]


def frame_id_from_sensor(sensor, config_key):
    config = sensor.get(config_key, {})
    ros_config = config.get("rosConfig", {})
    frame_id = ros_config.get("frameID", "")
    if frame_id:
        return frame_id

    ros2_config = config.get("ros2NativeConfig", {})
    return ros2_config.get("frameID", "")


def transform_from_sensor(sensor, config_key, parent_frame):
    child_frame = frame_id_from_sensor(sensor, config_key)
    if not child_frame:
        return None

    x, y, z = numeric_triplet(sensor["pos"], ("x", "y", "z"))
    roll, pitch, yaw = numeric_triplet(
        sensor["rot"], ("roll", "pitch", "yaw"), math.pi / 180.0
    )
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def load_transforms(sensor_config, parent_frame):
    with open(sensor_config, "r") as config_file:
        data = json.load(config_file)

    transforms = []
    for list_name, config_key, publish_param in SENSOR_GROUPS:
        if not rospy.get_param("~" + publish_param, False):
            continue

        for sensor in data.get(list_name, []):
            transform = transform_from_sensor(sensor, config_key, parent_frame)
            if transform is not None:
                transforms.append(transform)

    return transforms


def main():
    rospy.init_node("morai_sensor_static_tf")

    sensor_config = rospy.get_param("~sensor_config")
    sensor_config = os.path.abspath(os.path.expanduser(sensor_config))
    parent_frame = rospy.get_param("~parent_frame", "base_link")

    transforms = load_transforms(sensor_config, parent_frame)
    if not transforms:
        rospy.logerr("No static sensor transforms loaded from %s", sensor_config)
        raise SystemExit(1)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)

    rospy.loginfo("Loaded %d static sensor TFs from %s", len(transforms), sensor_config)
    for transform in transforms:
        translation = transform.transform.translation
        rospy.loginfo(
            "%s -> %s: xyz=(%.3f, %.3f, %.3f)",
            transform.header.frame_id,
            transform.child_frame_id,
            translation.x,
            translation.y,
            translation.z,
        )

    rospy.spin()


if __name__ == "__main__":
    main()
