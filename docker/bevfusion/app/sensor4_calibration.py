"""Parse MORAI Sensor_4 JSON and derive camera/LiDAR calibration matrices."""

import json
import hashlib
import math

import numpy as np


ROS_SENSOR_TO_OPTICAL = np.array(
    [[0.0, -1.0, 0.0, 0.0],
     [0.0, 0.0, -1.0, 0.0],
     [1.0, 0.0, 0.0, 0.0],
     [0.0, 0.0, 0.0, 1.0]],
    dtype=np.float64,
)


def load_sensor4_calibration(path, camera_order=None, intrinsic_source="horizontal_fov"):
    with open(path, "rb") as config_file:
        config_bytes = config_file.read()
    config = json.loads(config_bytes.decode("utf-8"))

    cameras_by_frame = {}
    for sensor in config.get("cameraList", []):
        camera_config = sensor.get("cc", {})
        ros_config = camera_config.get("rosConfig", {})
        frame_id = ros_config.get("frameID", "")
        if not frame_id:
            continue
        cameras_by_frame[frame_id] = (sensor, camera_config, ros_config)

    if camera_order is None:
        camera_order = list(cameras_by_frame)
    missing = [name for name in camera_order if name not in cameras_by_frame]
    if missing:
        raise ValueError("camera frames missing from sensor config: {}".format(missing))
    if len(camera_order) != 4:
        raise ValueError("BEVFusion Sensor_4 mode requires exactly four cameras")

    lidars = config.get("Lidar3DList", [])
    if len(lidars) != 1:
        raise ValueError("BEVFusion Sensor_4 mode requires exactly one 3D LiDAR")
    lidar = lidars[0]
    lidar_config = lidar.get("lc", {})
    lidar_ros = lidar_config.get("rosConfig", {})
    lidar_frame = lidar_ros.get("frameID", "")
    lidar_topic = lidar_ros.get("Topic", "")
    if not lidar_frame or not lidar_topic:
        raise ValueError("LiDAR frameID and Topic are required")

    base_from_lidar = pose_matrix(lidar["pos"], lidar["rot"])
    cameras = []
    for frame_id in camera_order:
        sensor, camera_config, ros_config = cameras_by_frame[frame_id]
        base_from_camera_ros = pose_matrix(sensor["pos"], sensor["rot"])
        camera_ros_from_lidar = np.linalg.inv(base_from_camera_ros) @ base_from_lidar
        lidar2cam = ROS_SENSOR_TO_OPTICAL @ camera_ros_from_lidar
        cam2lidar = np.linalg.inv(lidar2cam)
        cam2img = intrinsic_matrix(camera_config, intrinsic_source)
        cam2img4 = np.eye(4, dtype=np.float64)
        cam2img4[:3, :3] = cam2img
        lidar2img = cam2img4 @ lidar2cam
        cameras.append(
            {
                "frame_id": frame_id,
                "topic": ros_config.get("Topic", ""),
                "width": int(camera_config["cameraResWidth"]),
                "height": int(camera_config["cameraResHeight"]),
                "cam2img": cam2img,
                "lidar2cam": lidar2cam,
                "cam2lidar": cam2lidar,
                "lidar2img": lidar2img,
            }
        )

    return {
        "base_frame": "base_link",
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "intrinsic_source": intrinsic_source,
        "lidar_frame": lidar_frame,
        "lidar_topic": lidar_topic,
        "base_from_lidar": base_from_lidar,
        "cameras": cameras,
    }


def intrinsic_matrix(camera_config, source="horizontal_fov"):
    width = float(camera_config["cameraResWidth"])
    height = float(camera_config["cameraResHeight"])
    principal = camera_config.get("principalPoint", {})
    cx = float(principal.get("x", width * 0.5))
    cy = float(principal.get("y", height * 0.5))
    if source == "horizontal_fov":
        fov_radians = math.radians(float(camera_config["cameraFOV"]))
        focal = width / (2.0 * math.tan(fov_radians * 0.5))
        fx = fy = focal
    elif source == "focal_length_pixel":
        fx = fy = float(camera_config["focalLengthpixel"])
    else:
        raise ValueError("unsupported intrinsic source {!r}".format(source))
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def pose_matrix(position, rotation_degrees):
    roll = math.radians(float(rotation_degrees["roll"]))
    pitch = math.radians(float(rotation_degrees["pitch"]))
    yaw = math.radians(float(rotation_degrees["yaw"]))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [
        float(position["x"]),
        float(position["y"]),
        float(position["z"]),
    ]
    return transform


def serializable_calibration(calibration):
    return {
        "base_frame": calibration["base_frame"],
        "config_sha256": calibration["config_sha256"],
        "intrinsic_source": calibration["intrinsic_source"],
        "lidar_frame": calibration["lidar_frame"],
        "lidar_topic": calibration["lidar_topic"],
        "cameras": [
            {
                "frame_id": camera["frame_id"],
                "topic": camera["topic"],
                "width": camera["width"],
                "height": camera["height"],
                "cam2img": camera["cam2img"].tolist(),
                "lidar2cam": camera["lidar2cam"].tolist(),
                "cam2lidar": camera["cam2lidar"].tolist(),
                "lidar2img": camera["lidar2img"].tolist(),
            }
            for camera in calibration["cameras"]
        ],
    }
